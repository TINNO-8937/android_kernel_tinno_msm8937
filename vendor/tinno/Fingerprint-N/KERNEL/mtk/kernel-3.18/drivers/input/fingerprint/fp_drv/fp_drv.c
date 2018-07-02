/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "fp_drv.h"

///////////////////////////////////////////////////////////////////
static int fp_probe(struct platform_device *pdev);
static int fp_remove(struct platform_device *pdev);
///////////////////////////////////////////////////////////////////

static struct platform_driver fp_driver = {
	.probe = fp_probe,
	.remove = fp_remove,
	.driver = {
		   .name = "fp_drv",
	},
};

struct platform_device fp_device = {
    .name   	= "fp_drv",
    .id        	= -1,
};


//static struct fp_driver_t *g_fp_drv = NULL;
//static struct fp_driver_t fp_driver_list[MAX_DRV_NUM];

// IC info.
static char m_dev_name[64];
static int has_exist = 0;

// so & TA info.
static char m_dev_info[64];
static int all_info_exist = 0;
static int fp_id_pin_value = -1;

static DECLARE_WAIT_QUEUE_HEAD(waiter);

///////////////////////////////////////////////////////////////////

struct fp_gpio_data  {
    struct pinctrl *pinctrl1;
    struct pinctrl_state *id_in_low, *id_in_high;
};
struct fp_gpio_data *fp_gpio_id = NULL;

static void fp_id_set_value(struct fp_gpio_data *fp, int level) {
    if (level) {
        pinctrl_select_state(fp->pinctrl1, fp->id_in_high);
    } else {
        pinctrl_select_state(fp->pinctrl1, fp->id_in_low);
    }
}

int fp_id_parse_dt(struct device *dev, struct fp_gpio_data *pdata)
{
    struct device_node *node;
    int ret;
    int dt_error = 1;

    node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
    if (node) {		
        printk("[fp_id_parse_dt] mt_fp_pinctrl+++++++++++++++++\n");

        if (fp_gpio_id == NULL) {
            fp_gpio_id = kzalloc(sizeof(struct fp_gpio_data), GFP_KERNEL);
        }
		
        if(!fp_gpio_id) {
            klog("alloc fp_gpio_data fail.\n");
            return dt_error;
        }
		
        fp_gpio_id->pinctrl1 = devm_pinctrl_get(dev);
        if (IS_ERR(fp_gpio_id->pinctrl1)) {
            ret = PTR_ERR(fp_gpio_id->pinctrl1);
            klog("fwq Cannot find fp pinctrl1!\n");
            return ret;
        }

        fp_gpio_id->id_in_high = pinctrl_lookup_state(fp_gpio_id->pinctrl1, "id_in_high");
        if (IS_ERR(fp_gpio_id->id_in_high)) {
            ret = PTR_ERR(fp_gpio_id->id_in_high);
            klog("[fp_id_parse_dt] Cannot find fp pinctrl id_in_high!\n");
            return ret;
        }
		
        fp_gpio_id->id_in_low = pinctrl_lookup_state(fp_gpio_id->pinctrl1, "id_in_low");
        if (IS_ERR(fp_gpio_id->id_in_high)) {
            ret = PTR_ERR(fp_gpio_id->id_in_high);
            klog("[fp_id_parse_dt] Cannot find fp pinctrl id_in_high!\n");
            return ret;
        }

        return ret;		
    } else {
        klog("[fp_id_parse_dt] Can't find node mediatek,fingerprint----------\n");
        return dt_error;
    }
	return 0;
}

int read_fpId_pin_value(struct device *dev, char *label) {
    struct device_node *np = dev->of_node;
    int fp_pin = 0;
    int ret = -1;
    int val1 = -1, val2 = -1;

    if (fp_id_pin_value > 0) {
	 klog("%s: fingerprint id pin value :%d\n", __func__, fp_id_pin_value);
        return fp_id_pin_value;
    }

    ret = fp_id_parse_dt(dev, NULL);
    if (ret) {
        klog("%s: fp_id_parse_dt error!\n", __func__);
    }
    
    //fp_pin = of_get_named_gpio(np, label, 0);
    ret = of_property_read_u32_index(np, label, 1, &fp_pin);
    if (ret) {
        klog("%s: get %s fail, ret = %d\n", __func__, label, ret);
    }
	
    if (fp_pin <= 0) {
        klog("%s:fp pin gpio config err!\n", __func__);
        return -1;
    }

    // Give a HIGH
    //gpio_direction_input(fp_pin);
    //gpio_direction_output(fp_pin, 1);
    if (fp_gpio_id != NULL) {
        fp_id_set_value(fp_gpio_id, 1);
    }    
    val1 = __gpio_get_value(fp_pin);
    
    // Give a LOW
    //gpio_direction_input(fp_pin);
    //gpio_direction_output(fp_pin, 0);
    if (fp_gpio_id != NULL) {
        fp_id_set_value(fp_gpio_id, 0);
    }
    val2 = __gpio_get_value(fp_pin);
    
    klog("%s: (%d, %d, %d)\n", __func__, fp_pin, val1, val2);
    
    if (val1 == 1 && val2 == 0) {
        klog("#~~~~ High impedance!~~~~ \n");
        ret = __HIGH_IMPEDANCE;
    }
    else if (val1 == 0 && val2 == 0) {
        klog("#~~~~ LOW~~~~ \n");
        ret = __LOW;
    }
    else if (val1 == 1 && val2 == 1) {
        klog("~~~~ HIGH ~~~~ \n");
        ret = __HIGH;
    }
    else {
        klog("Err -what the fuck ??? \n");
        return -1;
    }

    // Set pin to input.
    //gpio_direction_input(fp_pin);
    fp_id_pin_value = ret;
    return ret;
}


int full_fp_chip_name(const char *name)
{
	__FUN();
	
	if((name == NULL) || (has_exist == 1))
	{
		klog("----(name == NULL) || (has_exist == 1)--err!---\n");
		return -1;
	}
	
	memset(m_dev_name, 0, sizeof(m_dev_name));
	strcpy(m_dev_name, name);
	has_exist = 1;	
	klog("---has_exist---:[%s]\n",  m_dev_name);
	return 0;
}

int full_fp_chip_info(const char *info)
{
	__FUN();

	if((info == NULL) || (all_info_exist == 1))
	{
		klog("----(info == NULL) || (all_info_exist == 1)--err!---\n");
		return -1;
	}

	memset(m_dev_info, 0, sizeof(m_dev_info));
	strcpy(m_dev_info, info);
	all_info_exist = 1;	
	klog("---m_dev_info---:[%s]\n",  m_dev_info);
	return 0;
}

static ssize_t info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(has_exist) 
	{
		return sprintf(buf, "%s", m_dev_name);    
	}
	return sprintf(buf, "%s", "unknow"); 
}

static ssize_t all_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(all_info_exist) 
	{
		return sprintf(buf, "%s", m_dev_info);    
	}
	return sprintf(buf, "%s", "unknow");   
}

static DEVICE_ATTR(fp_drv_info, 0444, info_show, NULL);
static DEVICE_ATTR(fp_drv_all_info, 0444, all_info_show, NULL);

static int fp_probe(struct platform_device *pdev)
{	
	__FUN();

	device_create_file(&pdev->dev, &dev_attr_fp_drv_info);
	device_create_file(&pdev->dev, &dev_attr_fp_drv_all_info);
	return 0;
}

static int fp_remove(struct platform_device *pdev)
{
	__FUN();
	device_remove_file(&pdev->dev, &dev_attr_fp_drv_info);
	device_remove_file(&pdev->dev, &dev_attr_fp_drv_all_info);
	return 0;
}

static int __init fp_drv_init(void)
{
	__FUN();

	if (platform_device_register(&fp_device) != 0) {
		klog( "device_register fail!.\n");
		return -1;
	
	}
	
	if (platform_driver_register(&fp_driver) != 0) {
		klog( "driver_register fail!.\n");
		return -1;
	}
	
	return 0;
}

static void __exit fp_drv_exit(void)
{
	__FUN();
	platform_driver_unregister(&fp_driver);
}

///////////////////////////////////////////////////////////////////
late_initcall(fp_drv_init);
module_exit(fp_drv_exit);

//MODULE_LICENSE("GPL");
//MODULE_DESCRIPTION("fp-drv");
//MODULE_AUTHOR("<mingyi.guo@tinno.com>");



