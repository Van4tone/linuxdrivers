/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright (C) 2015, Fuzhou Rockchip Electronics Co., Ltd
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *Description:  add a i2c driver for BT-GRM-2010-002 
 *Mail: 	    kane.shi@wpi-group.com 
 *Created Time: 2017-10-20 
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/adc.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/rk_keys.h>

// btgrm ges driver default
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#define     BTGRM_MAJOR  505
#define     BTGRM_NAME   "btgrm"
#define     BTGRM_CLASS_NAME "cls_btgrm"
#define     PRINT_BANK_REG_VALUE 0
#define     READ_REG_VALUE     0
#define     GES_ENTRY_TIME     500
#define     GES_QUIT_TIME      800
//#define     DEBUG_VALUE        0

//
#define EMPTY_DEFAULT_ADVALUE		1024
#define DRIFT_DEFAULT_ADVALUE		70
#define INVALID_ADVALUE			-1
#define EV_ENCALL			KEY_F4
#define EV_MENU				KEY_F1

#if 0
#define key_dbg(bdata, format, arg...)		\
	dev_info(&bdata->input->dev, format, ##arg)
#else
#define key_dbg(bdata, format, arg...)
#endif

#define DEBOUNCE_JIFFIES	(10 / (MSEC_PER_SEC / HZ))	/* 10ms */
#define ADC_SAMPLE_JIFFIES	(100 / (MSEC_PER_SEC / HZ))	/* 100ms */
#define WAKE_LOCK_JIFFIES	(1 * HZ)			/* 1s */
/***************btgrm func *********************/

struct class *cls;
struct i2c_client *btgrmclient;

void btgrm_read_reg_val(struct work_struct *work);
static DECLARE_DELAYED_WORK(btgrm_work, btgrm_read_reg_val);
void btgrm_read_reg_val(struct work_struct* work);
int btgrm_init(struct i2c_client* client);
/*****************btgrm driver msg**********************************/

enum rk_key_type {
	TYPE_GPIO = 1,
	TYPE_ADC
};

struct rk_keys_button {
	struct device *dev;
	u32 type;		/* TYPE_GPIO, TYPE_ADC */
	u32 code;		/* key code */
	const char *desc;	/* key label */
	u32 state;		/* key up & down state */
	int gpio;		/* gpio only */
	int adc_value;		/* adc only */
	int adc_state;		/* adc only */
	int active_low;		/* gpio only */
	int wakeup;		/* gpio only */
	struct timer_list timer;
};

struct rk_keys_drvdata {
	int nbuttons;
	/* flag to indicate if we're suspending/resuming */
	bool in_suspend;
	int result;
	int rep;
	int drift_advalue;
	struct wake_lock wake_lock;
	struct input_dev *input;
	struct delayed_work adc_poll_work;
	struct iio_channel *chan;
	struct rk_keys_button button[0];
};

static struct input_dev *sinput_dev;

void rk_send_power_key(int state)
{
	if (!sinput_dev)
		return;
	if (state) {
		input_report_key(sinput_dev, KEY_POWER, 1);
		input_sync(sinput_dev);
	} else {
		input_report_key(sinput_dev, KEY_POWER, 0);
		input_sync(sinput_dev);
	}
}
EXPORT_SYMBOL(rk_send_power_key);

void rk_send_wakeup_key(void)
{
	if (!sinput_dev)
		return;

	input_report_key(sinput_dev, KEY_WAKEUP, 1);
	input_sync(sinput_dev);
	input_report_key(sinput_dev, KEY_WAKEUP, 0);
	input_sync(sinput_dev);
}
EXPORT_SYMBOL(rk_send_wakeup_key);

static void keys_timer(unsigned long _data)
{
	struct rk_keys_button *button = (struct rk_keys_button *)_data;
	struct rk_keys_drvdata *pdata = dev_get_drvdata(button->dev);
	struct input_dev *input = pdata->input;
	int state;

	if (button->type == TYPE_GPIO)
		state = !!((gpio_get_value(button->gpio) ? 1 : 0) ^
			   button->active_low);
	else
		state = !!button->adc_state;

	if (button->state != state) {
		button->state = state;
		input_event(input, EV_KEY, button->code, button->state);
		key_dbg(pdata, "%skey[%s]: report event[%d] state[%d]\n",
			button->type == TYPE_ADC ? "adc" : "gpio",
			button->desc, button->code, button->state);
		input_event(input, EV_KEY, button->code, button->state);
		input_sync(input);
	}

	if (state)
		mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);
}

static irqreturn_t keys_isr(int irq, void *dev_id)
{
	struct rk_keys_button *button = (struct rk_keys_button *)dev_id;
	struct rk_keys_drvdata *pdata = dev_get_drvdata(button->dev);
	struct input_dev *input = pdata->input;

	BUG_ON(irq != gpio_to_irq(button->gpio));

	if (button->wakeup && pdata->in_suspend) {
		button->state = 1;
		key_dbg(pdata,
			"wakeup: %skey[%s]: report event[%d] state[%d]\n",
			(button->type == TYPE_ADC) ? "adc" : "gpio",
			button->desc, button->code, button->state);
		input_event(input, EV_KEY, button->code, button->state);
		input_sync(input);
	}
	if (button->wakeup)
		wake_lock_timeout(&pdata->wake_lock, WAKE_LOCK_JIFFIES);
	mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);

	return IRQ_HANDLED;
}

/*
static ssize_t adc_value_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);

	return sprintf(buf, "adc_value: %d\n", ddata->result);
}
static DEVICE_ATTR(get_adc_value, S_IRUGO | S_IWUSR, adc_value_show, NULL);
*/

static const struct of_device_id rk_key_match[] = {
	{ .compatible = "rockchip,key", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(of, rk_key_match);

static int rk_key_adc_iio_read(struct rk_keys_drvdata *data)
{
	struct iio_channel *channel = data->chan;
	int val, ret;

	if (!channel)
		return INVALID_ADVALUE;
	ret = iio_read_channel_raw(channel, &val);
	if (ret < 0) {
		pr_err("read channel() error: %d\n", ret);
		return ret;
	}
	return val;
}

static void adc_key_poll(struct work_struct *work)
{
	struct rk_keys_drvdata *ddata;
	int i, result = -1;

	ddata = container_of(work, struct rk_keys_drvdata, adc_poll_work.work);
	if (!ddata->in_suspend) {
		result = rk_key_adc_iio_read(ddata);
		if (result > INVALID_ADVALUE &&
		    result < (EMPTY_DEFAULT_ADVALUE - ddata->drift_advalue))
			ddata->result = result;
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = &ddata->button[i];

			if (!button->adc_value)
				continue;
			if (result < button->adc_value + ddata->drift_advalue &&
			    result > button->adc_value - ddata->drift_advalue)
				button->adc_state = 1;
			else
				button->adc_state = 0;
			if (button->state != button->adc_state)
				mod_timer(&button->timer,
					  jiffies + DEBOUNCE_JIFFIES);
		}
	}

	schedule_delayed_work(&ddata->adc_poll_work, ADC_SAMPLE_JIFFIES);
}

static int rk_key_type_get(struct device_node *node,
			   struct rk_keys_button *button)
{
	u32 adc_value;

	if (!of_property_read_u32(node, "rockchip,adc_value", &adc_value))
		return TYPE_ADC;
	else if (of_get_gpio(node, 0) >= 0)
		return TYPE_GPIO;
	else
		return -1;
}

static int rk_keys_parse_dt(struct rk_keys_drvdata *pdata,
			    struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child_node;
	struct iio_channel *chan;
	int ret, gpio, i = 0;
	u32 code, adc_value, flags, drift;

	if (of_property_read_u32(node, "adc-drift", &drift))
		pdata->drift_advalue = DRIFT_DEFAULT_ADVALUE;
	else
		pdata->drift_advalue = (int)drift;

	chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(chan)) {
		dev_info(&pdev->dev, "no io-channels defined\n");
		chan = NULL;
	}
	pdata->chan = chan;

	for_each_child_of_node(node, child_node) {
		if (of_property_read_u32(child_node, "linux,code", &code)) {
			dev_err(&pdev->dev,
				"Missing linux,code property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		pdata->button[i].code = code;
		pdata->button[i].desc =
		    of_get_property(child_node, "label", NULL);
		pdata->button[i].type =
		    rk_key_type_get(child_node, &pdata->button[i]);
		switch (pdata->button[i].type) {
		case TYPE_GPIO:
			gpio = of_get_gpio_flags(child_node, 0, &flags);
			if (gpio < 0) {
				ret = gpio;
				if (ret != -EPROBE_DEFER)
					dev_err(&pdev->dev,
						"Failed to get gpio flags, error: %d\n",
						ret);
				goto error_ret;
			}

			pdata->button[i].gpio = gpio;
			pdata->button[i].active_low =
			    flags & OF_GPIO_ACTIVE_LOW;
			pdata->button[i].wakeup =
			    !!of_get_property(child_node, "gpio-key,wakeup",
					      NULL);
			break;

		case TYPE_ADC:
			if (of_property_read_u32
			    (child_node, "rockchip,adc_value", &adc_value)) {
				dev_err(&pdev->dev,
					"Missing rockchip,adc_value property in the DT.\n");
				ret = -EINVAL;
				goto error_ret;
			}
			pdata->button[i].adc_value = adc_value;
			break;

		default:
			dev_err(&pdev->dev,
				"Error rockchip,type property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		i++;
	}

	return 0;

error_ret:
	return ret;
}

static int keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct rk_keys_drvdata *ddata = NULL;
	struct input_dev *input = NULL;
	int i, error = 0;
	int wakeup, key_num = 0;

	key_num = of_get_child_count(np);
	if (key_num == 0)
		dev_info(&pdev->dev, "no key defined\n");

	ddata = devm_kzalloc(dev, sizeof(struct rk_keys_drvdata) +
			     key_num * sizeof(struct rk_keys_button),
			     GFP_KERNEL);

	input = devm_input_allocate_device(dev);
	if (!ddata || !input) {
		error = -ENOMEM;
		return error;
	}
	platform_set_drvdata(pdev, ddata);
	dev_set_drvdata(&pdev->dev, ddata);

	input->name = "rk29-keypad";	/* pdev->name; */
	input->phys = "gpio-keys/input0";
	input->dev.parent = dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	ddata->input = input;

	/* parse info from dt */
	ddata->nbuttons = key_num;
	error = rk_keys_parse_dt(ddata, pdev);
	if (error)
		goto fail0;

	/* Enable auto repeat feature of Linux input subsystem */
	if (ddata->rep)
		__set_bit(EV_REP, input->evbit);

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, error: %d\n",
		       error);
		goto fail0;
	}
	sinput_dev = input;

	for (i = 0; i < ddata->nbuttons; i++) {
		struct rk_keys_button *button = &ddata->button[i];

		if (button->code) {
			setup_timer(&button->timer,
				    keys_timer, (unsigned long)button);
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, EV_KEY, button->code);
	}

	wake_lock_init(&ddata->wake_lock, WAKE_LOCK_SUSPEND, input->name);
	device_init_wakeup(dev, wakeup);

	for (i = 0; i < ddata->nbuttons; i++) {
		struct rk_keys_button *button = &ddata->button[i];

		button->dev = &pdev->dev;
		if (button->type == TYPE_GPIO) {
			int irq;

			error =
			    devm_gpio_request(dev, button->gpio,
					      button->desc ? : "keys");
			if (error < 0) {
				pr_err("gpio-keys: failed to request GPIO %d, error %d\n",
				       button->gpio, error);
				goto fail1;
			}

			error = gpio_direction_input(button->gpio);
			if (error < 0) {
				pr_err("gpio-keys: failed to configure input direction for GPIO %d, error %d\n",
				       button->gpio, error);
				gpio_free(button->gpio);
				goto fail1;
			}

			irq = gpio_to_irq(button->gpio);
			if (irq < 0) {
				error = irq;
				pr_err("gpio-keys: Unable to get irq number for GPIO %d, error %d\n",
				       button->gpio, error);
				gpio_free(button->gpio);
				goto fail1;
			}

			error = devm_request_irq(dev, irq, keys_isr,
						 button->active_low ?
						 IRQF_TRIGGER_FALLING :
						 IRQF_TRIGGER_RISING,
						 button->desc ?
						 button->desc : "keys",
						 button);
			if (error) {
				pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
				       irq, error);
				gpio_free(button->gpio);
				goto fail1;
			}
		}
	}

	input_set_capability(input, EV_KEY, KEY_WAKEUP);
	/* adc polling work */
	if (ddata->chan) {
		INIT_DELAYED_WORK(&ddata->adc_poll_work, adc_key_poll);
		schedule_delayed_work(&ddata->adc_poll_work,
				      ADC_SAMPLE_JIFFIES);
	}

	return error;

fail1:
	while (--i >= 0)
		del_timer_sync(&ddata->button[i].timer);
	device_init_wakeup(dev, 0);
	wake_lock_destroy(&ddata->wake_lock);
fail0:
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int keys_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(dev, 0);
	for (i = 0; i < ddata->nbuttons; i++)
		del_timer_sync(&ddata->button[i].timer);
	if (ddata->chan)
		cancel_delayed_work_sync(&ddata->adc_poll_work);
	input_unregister_device(input);
	wake_lock_destroy(&ddata->wake_lock);

	sinput_dev = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int keys_suspend(struct device *dev)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	ddata->in_suspend = true;
	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = ddata->button + i;

			if (button->wakeup)
				enable_irq_wake(gpio_to_irq(button->gpio));
		}
	}

	return 0;
}

static int keys_resume(struct device *dev)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = ddata->button + i;

			if (button->wakeup)
				disable_irq_wake(gpio_to_irq(button->gpio));
		}
		preempt_disable();
		/* for call resend_irqs, which may call keys_isr */
		if (local_softirq_pending())
			do_softirq();
		preempt_enable_no_resched();
	}

	ddata->in_suspend = false;

	return 0;
}

static const struct dev_pm_ops keys_pm_ops = {
	.suspend	= keys_suspend,
	.resume		= keys_resume,
};
#endif

static struct platform_driver keys_device_driver = {
	.probe		= keys_probe,
	.remove		= keys_remove,
	.driver		= {
		.name	= "rk-keypad",
		.owner	= THIS_MODULE,
		.of_match_table = rk_key_match,
#ifdef CONFIG_PM
		.pm	= &keys_pm_ops,
#endif
	}
};
/**************************btgrm i2c dirver*********************************/

int btgrm_write_byte(struct i2c_client * client,unsigned char reg,unsigned char val)
{
	int ret;
	unsigned char buf[2]={reg,val};

	struct i2c_msg msg[]={
		 {client->addr,0,2,buf},
	};
	ret = i2c_transfer(client->adapter,msg,ARRAY_SIZE(msg));
	if (ret < 0)
	{
		printk("btgrm write i2c_transfer fail!!!\n");
		return -EFAULT;
	}
	printk(" num of msgs:ret =%d\n",ret);
	return 0;
}
unsigned char btgrm_read_byte(struct i2c_client * client,unsigned char reg)
{
	 unsigned char buf[1]={reg};
	 unsigned char xbuf[1];
	 int ret;
	 struct i2c_msg msg[]={
		{client->addr,0,1,buf},
		{client->addr,1,1,xbuf},
	};
	 ret = i2c_transfer(client->adapter,msg,ARRAY_SIZE(msg));
	 if (ret < 0)
	 {
		 printk(" i2c_transfer fail!!!\n");
		 return -EFAULT;
	 }
//	 printk(" num of msgs: ret =%d\n",ret);
	 return xbuf[0];
}

/****
 *
 *传感器初始化函数，写入寄存器初始值。
 *
 */

int btgrm_init(struct i2c_client*client )
{
	int i,reg;
	char temp;
	char buf[2] = {};
// check the value of bank 0 
//	temp = btgrm_read_byte(client, BTGRM_REGISTER_BANK0);
//	printk("the value of BANK0 : %x!!!\n",temp);

	printk("START INIT SENSOR!!!!\n");
	mdelay(1000);
	//select bank 0
	temp = btgrm_read_byte(client, BTGRM_ADDR_GES_DET_FLAG_0); // test for read byte 
	printk("test for BTGRM_ADDR_GES_DET_FLAG_0  temp value is %x !!!\n",temp);

	reg = btgrm_write_byte(client, BTGRM_REGISTER_BANK0, 0);
	if(reg < 0)
	{
		printk(" reg=%d select bank0 failed!!!\n",reg);
	}
	mdelay(1000);
	temp = btgrm_read_byte(client, BTGRM_REGISTER_BANK0);
	printk("the value of BANK0 : %x!!!\n",temp);
	//
	buf[0]=btgrm_read_byte(client , 0 );
	buf[1]=btgrm_read_byte(client, 1);
	if((buf[0] != 0x20) ||( buf[1] != 0x76))
	{
		printk("Invalid address!!!\n");
	}
	if(buf[0] == 0x20)
	{
		printk("wake-up finish!!!\n");
	}
    else
	{
		printk(" buf[0] value is %x!!!\n",buf[0]);
	}
	for(i=0; i < INIT_REG_ARRAY_SIZE; i++)
	{
		btgrm_write_byte(client, init_register_array[i][0], init_register_array[i][1]);
	}

	temp = btgrm_read_byte(client, BTGRM_REGISTER_BANK0);
	printk("the value of BANK0__2 : %x!!!\n",temp);

//	check_regvalue(client);
	reg = btgrm_write_byte(client, BTGRM_REGISTER_BANK0, 0);
	if(reg < 0)
	{
		printk(" last select bank0 failed!!!\n");
	}

//	printk("Init SENSOR successed!!!\n");
	return 0;
}
/*************************send key to userspace**********/

/*************
*
* 工作队列，读取 0x43 寄存器的值
*
* ************/
void btgrm_read_reg_val(struct work_struct* work)
{
	int temp,temp2;
	temp = btgrm_read_byte(btgrmclient, BTGRM_ADDR_GES_DET_FLAG_0);
//	printk("temp value is %d !!!\n",temp);
	switch(temp)
	{
	case BTGRM_RIGHT_FLAG:
		
		{
		printk("right!!!\n");
		input_report_key(sinput_dev, KEY_PAGEUP, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_PAGEUP, 0);
		input_sync(sinput_dev);
		}
		break;
		mdelay(200);

	case BTGRM_LEFT_FLAG:
		printk("left!!!\n");
		input_report_key(sinput_dev, KEY_PAGEDOWN, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_PAGEDOWN, 0);
		input_sync(sinput_dev);
        break;
		mdelay(200);
	
	case BTGRM_DOWN_FLAG:
		printk("down!!!\n");
		input_report_key(sinput_dev, KEY_VOLUMEDOWN, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_VOLUMEDOWN, 0);
		input_sync(sinput_dev);
		break;
		mdelay(200);

	case BTGRM_UP_FLAG:
		printk("up!!!\n");
		input_report_key(sinput_dev, KEY_VOLUMEUP, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_VOLUMEUP, 0);
		input_sync(sinput_dev);
		break;
		mdelay(200);

	case BTGRM_BACKWARD_FLAG:
		printk("back!!!\n");
		input_report_key(sinput_dev, KEY_1, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_1, 0);
		input_sync(sinput_dev);
		break;
		mdelay(200);

	case BTGRM_FORWARD_FLAG:
		printk("forward!!!\n");
		input_report_key(sinput_dev, KEY_2, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_2, 0);
		input_sync(sinput_dev);
		break;
		mdelay(200);

	case BTGRM_CLOCKWISE_FLAG:
	//	rk_send_power_key(1);
	//	rk_send_power_key(0);
		printk("clockwise!!!\n");
		input_report_key(sinput_dev, KEY_3, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_3, 0);
		input_sync(sinput_dev);
		break;
		mdelay(200);

	case BTGRM_COUNTER_CLOCKWISE_FLAG:
		printk("count_clockwise!!!\n");
		input_report_key(sinput_dev, KEY_4, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_4, 0);
		input_sync(sinput_dev);
        break;
		mdelay(200);

	default:
		break;
		mdelay(200);
	}
		temp2 = btgrm_read_byte(btgrmclient,BTGRM_ADDR_GES_DET_FLAG_1);
//		printk(" temp2 value is %d !!!\n",temp2);
		switch(temp2)
		{
		case BTGRM_WAVE_FLAG:
		printk("wave!!!\n");
		input_report_key(sinput_dev, KEY_CAMERA, 1);
		input_sync(sinput_dev);
		input_report_key(sinput_dev, KEY_CAMERA, 0);
		input_sync(sinput_dev);
		break;
		mdelay(500);

		default:
		break;
		mdelay(200);
		}		
	schedule_delayed_work(&btgrm_work,1*HZ);
}
int btgrm_open(struct inode *inode, struct file *file)
{
	printk("btgrm_open !!!\n");
	return 0;
}

int btgrm_release(struct inode *inode, struct file *file)
{
	printk("btgrm_release!!!\n");
	return 0;
}

const struct file_operations btgrm_fops={
	 .owner = THIS_MODULE,
	 .open = btgrm_open,
	 .release = btgrm_release,
};

const struct i2c_device_id btgrm_id_table[]={
	 {
		  "btgrm",0
	 },
	 {
		  
	 },
};
const struct of_device_id btgrm_dt_table[]={
	 {
		  .compatible = "btgrm",
	 },
	 {
		  
	 },
};

int btgrm_probe(struct i2c_client *client, const struct i2c_device_id *btgrm)
{
	
	btgrmclient = client;
	register_chrdev(BTGRM_MAJOR, BTGRM_NAME, &btgrm_fops);

	cls = class_create(THIS_MODULE, BTGRM_CLASS_NAME);
	device_create(cls, NULL, MKDEV(BTGRM_MAJOR, 0), NULL, "btgrm");
	btgrm_init(btgrmclient);

	schedule_delayed_work(&btgrm_work,1*HZ);
	return 0;
}

int btgrm_remove(struct i2c_client *client)
{
	 printk(" btgrm_remove!!!\n");
	 cancel_delayed_work(&btgrm_work);

	 device_destroy(cls,MKDEV(BTGRM_MAJOR,0));
	 class_destroy(cls);
	 unregister_chrdev(BTGRM_MAJOR,BTGRM_NAME);
	 return 0;

}

struct i2c_driver btgrm_driver={
	 .driver={
		  .name = "btgrm",
		  .of_match_table = btgrm_dt_table,
	 },
	 .id_table = btgrm_id_table,
	 .probe    = btgrm_probe,
	 .remove   = btgrm_remove,
};

static int __init rk_keys_driver_init(void)
{
	 platform_driver_register(&keys_device_driver);
	return i2c_add_driver(&btgrm_driver);
}

static void __exit rk_keys_driver_exit(void)
{
	platform_driver_unregister(&keys_device_driver);
	i2c_del_driver(&btgrm_driver);
}

late_initcall_sync(rk_keys_driver_init);
module_exit(rk_keys_driver_exit);
