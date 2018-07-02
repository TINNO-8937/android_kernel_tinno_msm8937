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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
//#include <mach/mt_spi.h>
//#include <mach/mt_gpio.h>
//#include <mach/eint.h>
#include <mach/gpio_const.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/compat.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/of_irq.h>
//add by bacon for fp_wake_lock
#include <linux/kobject.h>
#include <linux/debugfs.h>
#include <../kernel/power/power.h>
//add by bacon for fp_wake_lock
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#define VERBOSE  0
#include <asm/uaccess.h>
#define SL_MAX_FRAME_NUM 2

#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include "mt_spi.h"
#include "mt_spi_hal.h"
#include <mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/irqchip/mt-eic.h>
//#include "../../../spi/mediatek/mt6755/mt_spi.h"
//#include "../../../spi/mediatek/mt6755/mt_spi_hal.h"

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/fb.h>

#include "../fp_drv/fp_drv.h"
#include "slspi_tee.h"

#define  SL_USE_PLATFORM_BUS     1
//#define  SL_USE_SPI_BUS	1

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
//#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	

#define SILEAD_FP_NAME "silead_fp"

static DECLARE_BITMAP(minors, N_SPI_MINORS);
/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

//static char tmp[1024];
#define EINT_GPIO_NUM GPIO48
#define SPI_RESET_PIN GPIO75	
#define GPIO_FP_EINT_PIN_M_EINT GPIO_MODE_GPIO
#define MTK_SPEED_BASE 1000000000

#define SPI_POWER_PIN GPIO57

#define SL_LOG_TAG "SLCODE"
#undef SL_LOGD
#ifndef SL_LOGD
#define SL_LOGD(fmt,args...)    \
        printk(KERN_WARNING SL_LOG_TAG "||%-40s ||%-6d "fmt"\n",__func__,__LINE__,##args)
#endif

#define REDUCE_REPEAT_IRQ

//#define sileadDBG

#ifdef sileadDBG
static void intToStr(unsigned int chipID,char * buf);
#endif

//static struct spi_transfer	t[SL_ONE_FRAME_PAGES];
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#define SL_VDD_MIN_UV      2800000
#define SL_VDD_MAX_UV      2800000
#define SL_VIO_MIN_UV      1750000
#define SL_VIO_MAX_UV      1950000

static int irq_enabled = 0;
static int irq_requested = 0;
static int irq_counter = 0;
static int isPowerOn = 0;

static int finger_status = 0;

static struct spidev_data	*fp_spidev = NULL;
static unsigned int spidev_major = 0;
static struct cdev spicdev;

//#ifndef REDUCE_REPEAT_IRQ
static int g_irq_svc_debounce = 0;
//#endif
//add by matthew start
static int chip_power_off = 0;
//add by matthew end

static unsigned bufsiz = 4096;
//static inline ssize_t sl_fp_write(struct spidev_data *spidev, uint8_t reg, uint32_t w_data);
//add by bacon for fp_wake_lock
static ssize_t fp_wake_lock_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n);
static ssize_t fp_wake_unlock_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t n);
static ssize_t fp_wake_lock_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf);
static ssize_t fp_wake_unlock_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf);
//add by bacon for fp_wake_lock

#if defined(SL_USE_PLATFORM_BUS)
static int spidev_probe(struct platform_device *spi);
static int spidev_remove(struct platform_device *spi);
#elif defined(SL_USE_SPI_BUS)
static int spidev_probe(struct spi_device *spi);
static int spidev_remove(struct spi_device *spi);
#endif
static int silead_request_irq(struct spidev_data *pdata);
static int silead_power_ctl(struct spidev_data *pdata, int on);
static void silead_spi_clk_enable(struct spidev_data *pdata, int bonoff);
static int spidev_shutdown_hw(struct spidev_data *spidev);

enum {
    FP_VENDOR_INVALID = 0,
    FPC_VENDOR,
    ELAN_VENDOR,
    GOODIX_VENDOR,
    SILEAD_VENDOR,
};

extern int get_fp_vendor(void);

//for lib version
#define SL_MAX_LIB_BUF 64
static char sl_lib_ver_buf[SL_MAX_LIB_BUF] = "unknow";

static struct pinctrl *pinctrl1 = NULL;
static struct pinctrl_state *eint_as_int = NULL;
static struct pinctrl_state *eint_in_low = NULL;
static struct pinctrl_state *eint_in_float = NULL;
static struct pinctrl_state *fp_rst_low = NULL;
static struct pinctrl_state *fp_rst_high = NULL;

static struct fasync_struct *fasync_queue = NULL;
static wait_queue_head_t silead_poll_wq;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");


#define SL_READ  0x00 
#define SL_WRITE 0xFF 

//add by bacon for fp_wake_lock
#define silead_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0666,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}


silead_attr(fp_wake_lock);
silead_attr(fp_wake_unlock);

static struct attribute * g[] = {
	&fp_wake_lock_attr.attr,
	&fp_wake_unlock_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct wakelock {
	char			*name;
	struct wakeup_source	ws;
};

static struct wakelock * g_wakelock_list[10] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

static DEFINE_MUTEX(wakelocks_lock);

static ssize_t fp_wake_lock_show(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      char *buf)
{
	//return pm_show_wakelocks(buf, true);
	int i;
	char *str = buf;
	char *end = buf + PAGE_SIZE;

	mutex_lock(&wakelocks_lock);
    
    
	for(i=0;i<10;i++)
    {
        if(g_wakelock_list[i]!=NULL)
		{
	        str += scnprintf(str, end - str, "%s ", g_wakelock_list[i]->name);
        }
	}
	if (str > buf)
		str--;

	str += scnprintf(str, end - str, "\n");

	mutex_unlock(&wakelocks_lock);
	return (str - buf);
}

static ssize_t fp_wake_lock_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
//	int error = pm_wake_lock(buf);
//	return error ? error : n;

	int i, j;
	int ret= -1;
    char * wl_name;
    struct wakelock *wl;

	wl_name = kstrndup(buf, n, GFP_KERNEL);
	if (!wl_name) {
		return -ENOMEM;
	}

	mutex_lock(&wakelocks_lock);
	for(j=0; j<10; j++)
	{
		if(g_wakelock_list[j]!=NULL)
		{
			if(strcmp(g_wakelock_list[j]->name,buf) == 0)
			{
                wl = g_wakelock_list[j];
				ret = n;
				break;
			}
		}
	}

    if (j == 10)
    {
		wl = kzalloc(sizeof(*wl), GFP_KERNEL);
		if (!wl)
			return -ENOMEM;

		wl->name = wl_name;
		wl->ws.name = wl_name;
		wakeup_source_add(&wl->ws);

       	for(i=0; i<10; i++)
		{
			if(g_wakelock_list[i]==NULL)
			{
				g_wakelock_list[i] = wl;
				ret = n;
				break;
			}
		}
    }

    __pm_stay_awake(&wl->ws);
	mutex_unlock(&wakelocks_lock);

    SL_LOGD("fp_wake_lock_store ret = %d\n", ret);
	return ret;
}

static ssize_t fp_wake_unlock_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	//return pm_show_wakelocks(buf, fasle);
	int i;
	char *str = buf;
	char *end = buf + PAGE_SIZE;

	mutex_lock(&wakelocks_lock);
    
    
	for(i=0;i<10;i++)
    {
        if(g_wakelock_list[i]!=NULL)
		{
	        str += scnprintf(str, end - str, "%s ", g_wakelock_list[i]->name);
        }
	}
	if (str > buf)
		str--;

	str += scnprintf(str, end - str, "\n");

	mutex_unlock(&wakelocks_lock);
	return (str - buf);
}

static ssize_t fp_wake_unlock_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t n)
{
//	int error = pm_wake_unlock(buf);
//	return error ? error : n;

	struct wakelock *wl;
	int ret = -1;

	int i;

	mutex_lock(&wakelocks_lock);

	for(i=0;i<10;i++)
	{
		if(g_wakelock_list[i]!=NULL)
		{
			if(strcmp(g_wakelock_list[i]->name,buf)==0)
			{
				wl = g_wakelock_list[i];
                __pm_relax(&wl->ws);
				wakeup_source_remove(&wl->ws);
				kfree(wl->name);
				kfree(wl);
				g_wakelock_list[i] = NULL;
				ret = n;
				break;
			}
		}
	}

	mutex_unlock(&wakelocks_lock);
    SL_LOGD("fp_wake_unlock_store ret = %d\n", ret);
	return ret;
}
//add by bacon for fp_wake_unlock

static void spidev_irq_work(struct work_struct *work)
{
	struct spidev_data*		spidev = container_of(work,struct spidev_data,int_work);
	char*					env_ext[2] = {"SILEAD_FP_EVENT=IRQ", NULL};
//#ifndef REDUCE_REPEAT_IRQ
	char*					env_ext_forged[2] = {"SILEAD_FP_EVENT=IRQ_FORGED", NULL};
//#endif
	//SL_LOGD("irq bottom half spidev_irq_work enter \n");
//#ifndef REDUCE_REPEAT_IRQ
    if(g_irq_svc_debounce)
    {
        kobject_uevent_env(&spidev->spi->dev.kobj, KOBJ_CHANGE, env_ext_forged);
        return;
    }
    g_irq_svc_debounce = 1;
//#endif

	kobject_uevent_env(&spidev->spi->dev.kobj, KOBJ_CHANGE, env_ext); 	
}

static irqreturn_t spidev_irq_routing(int irq, void* dev)
{
	struct spidev_data *spidev = fp_spidev;
	
	//add by matthew start
	if(chip_power_off)
	{
		return IRQ_HANDLED;
	}
	//add by matthew end
	
#ifdef REDUCE_REPEAT_IRQ
	spidev_shutdown_hw(spidev);
#endif
	disable_irq_nosync(spidev->int_irq);
	irq_counter = 0;
	irq_enabled = 0;

	if(spidev->wqueue)
	{
		#ifdef sileadDBG
		SL_LOGD("now spidev->wqueue is not NULL!\n");
		#endif
		queue_work(spidev->wqueue,&spidev->int_work);
	}
	else
	{
		#ifdef sileadDBG
		SL_LOGD("now spidev->wqueue is NULL!\n");
		#endif
		schedule_work(&spidev->int_work);
	}

	return IRQ_HANDLED;
} 
/*-------------------------------------------------------------------------*/

void silead_irq_enable(struct spidev_data *pdata)
{	
	unsigned long irqflags = 0;
	SL_LOGD("IRQ Enable = %d.\n", pdata->int_irq);

	if(irq_requested == 0)
	{
		silead_request_irq(pdata);
	}
  
	spin_lock_irqsave(&pdata->spi_lock, irqflags);
	if (irq_enabled == 0 && irq_counter == 0) 
	{
		irq_counter = 1;
		enable_irq(pdata->int_irq);
		irq_enabled = 1;
		//#ifndef REDUCE_REPEAT_IRQ
		g_irq_svc_debounce = 1;
        mdelay(5);
		g_irq_svc_debounce = 0;
        //#endif
	}
	spin_unlock_irqrestore(&pdata->spi_lock, irqflags);
}

void silead_irq_disable(struct spidev_data *pdata)
{
	unsigned long irqflags;
	SL_LOGD("IRQ Disable = %d.\n", pdata->int_irq);

	if(irq_requested == 0)
	{
		return;
	}
	
	spin_lock_irqsave(&pdata->spi_lock, irqflags);
	if (irq_enabled && irq_counter>0)
	{
		irq_counter = 0;
		disable_irq(pdata->int_irq);
		irq_enabled = 0; 
	}
	spin_unlock_irqrestore(&pdata->spi_lock, irqflags);
}

static int silead_request_irq(struct spidev_data *pdata)
{    
    int err;
    int irq_flags;
    
    if(irq_requested)
   	{
   		return 0;
    }
    
    irq_requested = 1;
    irq_enabled = 0;
    irq_counter = 0;
    spin_lock_init(&pdata->spi_lock);

    irq_flags = IRQF_NO_SUSPEND |  IRQF_TRIGGER_RISING;
    SL_LOGD("%s  Interrupt  %d  wake up is %d" 
					"irq flag is 0x%X\n", 
					__func__,pdata->int_irq, pdata->wakeup,irq_flags);
	
    err = request_irq(pdata->int_irq, spidev_irq_routing, irq_flags, "sl_wake_up", pdata);
	
    if (err) {
        SL_LOGD("Failed to request IRQ %d.\n", err);
        irq_requested = 0;
        return -1;
    }
    
    enable_irq_wake(pdata->int_irq);
    disable_irq(pdata->int_irq);
    return 0;
}

static void silead_reset_output(struct spidev_data *spidev, int level)
{
	if (level)
		pinctrl_select_state(pinctrl1, fp_rst_high);
	else
		pinctrl_select_state(pinctrl1, fp_rst_low);
}

static int spidev_reset_hw(struct spidev_data *spidev)
{
	silead_reset_output(spidev, 0);
	mdelay(1);
	silead_reset_output(spidev, 1);
	return 0;
}

static int spidev_shutdown_hw(struct spidev_data *spidev)
{
	silead_reset_output(spidev, 0);
	return 0;
}

static long spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int						retval = 0;
    struct spidev_data		*spidev;
 
    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC){
	
	return -ENOTTY; }

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spidev = filp->private_data;

    mutex_lock(&spidev->buf_lock);

    switch (cmd) {
		case SPI_HW_RESET :
			spidev_reset_hw(spidev);
			//add by matthew start
			if(chip_power_off)
			{
				mdelay(1);
				chip_power_off = 0;
			}
			//add by matthew end
			break;

		case SPI_HW_SHUTDOWN:
			//add by matthew start
			chip_power_off = 1;
			//add by matthew end
			
			spidev_shutdown_hw(spidev);
			
			//add by matthew start
			silead_irq_enable(spidev);
			//add by matthew end
			break;

		case SPI_CLOSE_CLOCK:
			silead_spi_clk_enable(spidev, 0);
				break;
				
		case SPI_OPEN_CLOCK:
			silead_spi_clk_enable(spidev, 1);
				break;

		case SPI_HW_POWEROFF:
			SL_LOGD("SPI_HW_POWEROFF called\n");
			silead_power_ctl(spidev, 0);
			break;

		case SPI_HW_POWERON:
			SL_LOGD("SPI_HW_POWERON called\n");
			silead_power_ctl(spidev, 1);
			break;
            
		case SPI_HW_SET_APP_VER:
			{
				int ret = copy_from_user(sl_lib_ver_buf, (char *)arg, SL_MAX_LIB_BUF);
				if (!ret) {
					sl_lib_ver_buf[SL_MAX_LIB_BUF-1] = '\0';
					full_fp_chip_info(sl_lib_ver_buf);
				}
			}
			break;

		case SPI_HW_IRQ_ENBALE:
			if(arg)
			{
				silead_irq_enable(spidev);
			}
			else
			{
				silead_irq_disable(spidev);
			}
			break;
			
		case SPI_HW_IRQ_REQUEST:
			silead_request_irq(spidev);
			break;

		case SPI_HW_FINGER_STATE_INFO:
			if(arg)
			{
				SL_LOGD("finger on\n");
				finger_status = 1;
			}
			else
			{
				SL_LOGD("finger off\n");
				finger_status = 2;
			}

			if(fasync_queue)
			{
				kill_fasync(&fasync_queue, SIGIO, POLL_IN);
			}	
			break;

		case IOCTL_FINGER_STATE_INFO:
			if(arg)
			{
				retval = __put_user(finger_status, (__u32 __user *)arg);
			}
			break;


		default:
			break;
    }

    mutex_unlock(&spidev->buf_lock);
	
    return retval;
}

static int silead_power_ctl(struct spidev_data *pdata, int on)
{
	int rc = 0;

	if (on && (!isPowerOn)) {
		rc = regulator_enable(pdata->vdd);
		if (rc) {
			SL_LOGD("SLCODE Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
		
#ifdef SLFP_VIO_CTRL
		rc = regulator_enable(pdata->vio);
		if (rc) {
			SL_LOGD("SLCODE Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(pdata->vdd);
			return rc;
		}
#endif
		msleep(10);

		isPowerOn = 1;
		SL_LOGD(" set PowerOn ok !\n");
	} else if (!on && (isPowerOn)) {

		rc = regulator_disable(pdata->vdd);
		if (rc) {
			SL_LOGD("SLCODE Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

#ifdef SLFP_VIO_CTRL
		rc = regulator_disable(pdata->vio);
		if (rc) {
			SL_LOGD("SLCODE Regulator vio disable failed rc=%d\n", rc);
		}
#endif

		isPowerOn = 0;
		SL_LOGD(" set PowerDown !ok \n");
	} else {
		SL_LOGD(	"SLCODE Ignore power status change from %d to %d\n",
				on, isPowerOn);
	}
	return rc;
}

int silead_power_init(struct spidev_data *pdata)
{
	int ret = 0;

	pinctrl1 = NULL;
	eint_as_int = NULL;
	eint_in_low = NULL;
	eint_in_float = NULL;
	fp_rst_low = NULL;
	fp_rst_high = NULL;
	
	pdata->spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");

	pdata->vdd = regulator_get(&pdata->spi->dev, "vfp");
	if (IS_ERR(pdata->vdd)) {
		ret = PTR_ERR(pdata->vdd);
		SL_LOGD("SLCODE Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}
	
	if (regulator_count_voltages(pdata->vdd) > 0) {
		ret = regulator_set_voltage(pdata->vdd, SL_VDD_MIN_UV,
					   SL_VDD_MAX_UV);
		if (ret) {
			SL_LOGD("SLCODE Regulator set_vtg failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}
	
#ifdef SLFP_VIO_CTRL
	pdata->vio = regulator_get(&pdata->spi->dev, "vio");
	if (IS_ERR(pdata->vio)) {
		ret = PTR_ERR(pdata->vio);
		SL_LOGD("SLCODE Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(pdata->vio) > 0) {
		ret = regulator_set_voltage(pdata->vio,
				SL_VIO_MIN_UV,
				SL_VIO_MAX_UV);
		if (ret) {
			SL_LOGD("SLCODE Regulator set_vtg failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}
#endif
	
	SL_LOGD("SLCODE Regulator set_vtg OK vdd ret=%d \n", ret);
	return 0;
	
#ifdef SLFP_VIO_CTRL
reg_vio_put:
	regulator_put(pdata->vio);
reg_vdd_set_vtg:
	if (regulator_count_voltages(pdata->vdd) > 0)
		regulator_set_voltage(pdata->vdd, 0, SL_VDD_MAX_UV);
#endif
reg_vdd_put:
	regulator_put(pdata->vdd);
	return ret;
}

int silead_power_deinit(struct spidev_data *pdata)
{
    int ret = 0;

    if (pdata->vdd)
    {   
        if (regulator_count_voltages(pdata->vdd) > 0)
            regulator_set_voltage(pdata->vdd, 0, SL_VDD_MAX_UV);
        
        regulator_disable(pdata->vdd);
        isPowerOn = 0;
        
        regulator_put(pdata->vdd);
    }
    
#ifdef SLFP_VIO_CTRL
    if (pdata->vio)
    {   
        if (regulator_count_voltages(pdata->vio) > 0)
            regulator_set_voltage(pdata->vio, 0, SL_VIO_MAX_UV);
        
        regulator_disable(pdata->vio);
        regulator_put(pdata->vio);
    }
#endif 
	
    return ret;
}

static void silead_spi_clk_enable(struct spidev_data *pdata, int bonoff)
{
#ifdef CONFIG_MTK_CLKMGR
	if (bonoff)
		enable_clock(MT_CG_PERI_SPI0, "spi");
	else
		disable_clock(MT_CG_PERI_SPI0, "spi");

#else
	/* changed after MT6797 platform */
	struct mt_spi_t *ms = NULL;
	ms = spi_master_get_devdata(fp->pdev->master);

	if (bonoff) {
		mt_spi_enable_clk(ms);
	} else {
		mt_spi_disable_clk(ms);
	}
#endif
}

static void silead_gpio_as_int(struct spidev_data* spidev)
{
	SL_LOGD("silead_gpio_as_int\n");
	pinctrl_select_state(pinctrl1, eint_as_int);
}

static char silead_gpio_config(struct spidev_data* spidev)
{	
	silead_reset_output(spidev, 0);
	mdelay(3);
	silead_reset_output(spidev, 1);
	mdelay(3);

	struct device_node *node;
	SL_LOGD("[silead]:%s enter\n", __func__);
	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	if ( node)
	{
		silead_gpio_as_int(spidev);		
		spidev->int_irq = irq_of_parse_and_map( node, 0);
		SL_LOGD("silead pdata->int_irq = %d\n",  spidev->int_irq);
		if (! spidev->int_irq)
		{
			SL_LOGD("silead irq_of_parse_and_map fail!!\n");
			return -1;
		}
	}
	else
	{
		SL_LOGD("silead null irq node!!\n");
		return -1;
	}

	return 0;
}

static int silead_parse_dt(struct device *dev, struct spidev_data *pdata)
{
	struct device_node *node;
	int err = 0;
	
	pinctrl1 = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl1)) {
		err = PTR_ERR(pinctrl1);
		dev_err(dev, "Cannot find fp pinctrl1!\n");
		return err;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	if (node) {
		int ret = 0;

		fp_rst_high = pinctrl_lookup_state(pinctrl1, "fp_rst_high");
		if (IS_ERR(fp_rst_high)) {
			ret = PTR_ERR(fp_rst_high);
			dev_err(&pdata->spi->dev, "Cannot find fp pinctrl fp_rst_high!\n");
		}
		fp_rst_low = pinctrl_lookup_state(pinctrl1, "fp_rst_low");
		if (IS_ERR(fp_rst_low)) {
			ret = PTR_ERR(fp_rst_low);
			dev_err(&pdata->spi->dev, "Cannot find fp pinctrl fp_rst_low!\n");
		}
		eint_as_int = pinctrl_lookup_state(pinctrl1, "eint_as_int");
		if (IS_ERR(eint_as_int)) {
			ret = PTR_ERR(eint_as_int);
			dev_err(&pdata->spi->dev, "Cannot find fp pinctrl eint_as_int!\n");
		}
		eint_in_low = pinctrl_lookup_state(pinctrl1, "eint_in_low");
		if (IS_ERR(eint_in_low)) {
			ret = PTR_ERR(eint_in_low);
			dev_err(&pdata->spi->dev, "Cannot find fp pinctrl eint_output_low!\n");
		}
		eint_in_float = pinctrl_lookup_state(pinctrl1, "eint_in_float");
		if (IS_ERR(eint_in_float)) {
			ret = PTR_ERR(eint_in_float);
			dev_err(&pdata->spi->dev, "Cannot find fp pinctrl eint_output_high!\n");
		}
		SL_LOGD("silead_parse_dt ret=%d\n", ret);

		return ret;

	}else{
		SL_LOGD("of_find_compatible_node error\n");
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = fp_spidev;
	return 0;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;
#ifdef sileadDBG
static void intToStr(unsigned int chipID,char * buf)
{
	int i;
	for(i=0;i<8;i++){
		if(((chipID >> 4*(7-i)) & 0xf)>=0 && ((chipID >> 4*(7-i)) & 0xf) <=9)
			buf[i]=((chipID >> 4*(7-i)) & 0xf) + '0';
		else
			buf[i]=(((chipID >> 4*(7-i)) & 0xf) -10)+ 'a';
	}
	buf[8]='\0';
	return ;
}
#endif

#if defined(SL_USE_PLATFORM_BUS)
static unsigned int spidev_poll(struct file *file, poll_table *wait)
{
	int mask=0; 

	poll_wait(file, &silead_poll_wq, wait);
	return mask;
}

static int spidev_fp_fasync(int fd, struct file * filp, int on)
{
	return fasync_helper(fd, filp, on, &fasync_queue);
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	//.write =	spidev_write,
	//.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,	
	.poll			= spidev_poll,
	.fasync 		= spidev_fp_fasync,
};

#elif defined(SL_USE_SPI_BUS)

static int spidev_mmap(struct file* filep, struct vm_area_struct *vma)
{
    struct spidev_data	*spidev = filep->private_data;

    vma->vm_flags |= VM_RESERVED;
    vma->vm_flags |= VM_LOCKED;
    if (NULL == spidev->mmap_buf) {
        dev_err(&spidev->spi->dev,"frame buffer is not alloc\n");
        return -ENOMEM;
    }
    return remap_pfn_range( vma, vma->vm_start,
                            virt_to_phys((void*)((unsigned long)spidev->mmap_buf))>>PAGE_SHIFT,
                            vma->vm_end - vma->vm_start, PAGE_SHARED);
}

static const struct file_operations spidev_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	spidev_write,
    .read =		spidev_read,
    .unlocked_ioctl = spidev_ioctl,
    .compat_ioctl = spidev_compat_ioctl,
    .open =		spidev_open,
    .release =	spidev_release,
    .llseek =	no_llseek,
    .mmap = spidev_mmap,
};
#endif

#if defined(SL_USE_PLATFORM_BUS)
static const struct dev_pm_ops silead_fp_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};

#ifdef CONFIG_OF
static const struct of_device_id silead_of_match[] = {
	{ .compatible = "mediatek,fingerprint", },
	{ .compatible = "mediatek,silead-fp", },
	{ .compatible = "silead,silead-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, silead_of_match);
#else
#define silead_of_match NULL
#endif


static const struct platform_device_id platform_silead_id[] = {
	{"silead_fp", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, spi_silead_id);

static struct platform_driver silead_fp_driver = {
	.driver = {
		.name 	= "silead_fp",
		.owner = THIS_MODULE,
		//.pm 	= &silead_fp_pm_ops,
		.of_match_table = silead_of_match,
	},
	.probe 	= spidev_probe,
	.remove = spidev_remove,
	.id_table = platform_silead_id,
};

#elif defined(SL_USE_SPI_BUS)

struct spi_device_id spi_silead_id = {"silead_fp",0};

struct of_device_id silead_of_match[] = {
	{ .compatible = "mediatek, silead_fp"},
	{},
};

static struct spi_driver spidev_spi_driver = {
    .driver = {
        .name =		"silead_fp",   //spidev
		.bus = &spi_bus_type,
        .owner =	THIS_MODULE,
    },
    .probe =	spidev_probe,
    .remove =	spidev_remove,    /*__devexit_p*/

//   .suspend = spidev_suspend,
//   .resume  = spidev_resume,
	.id_table = &spi_silead_id,
    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/
struct mt_chip_conf chip_config = {    //修改配置参数  
		.setuptime =10,//6, //10,//15,//10 ,//3,
		.holdtime =10, //6,//10,//15,//10,//3,
		.high_time =30,//4,//6,//8,//12, //25,//8,      //10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
		.low_time = 30,//4,//6,//8,//12,//25,//8,
		.cs_idletime = 2,//30,// 60,//100,//12,
		.ulthgh_thrsh = 0,

		.rx_mlsb = SPI_MSB, 
		.tx_mlsb = SPI_MSB,		 
		.tx_endian = 0,
		.rx_endian = 0,

		.cpol = SPI_CPOL_0,
		.cpha = SPI_CPHA_0,        //修改0值就可以修改到对应的模式
		
		.com_mod = DMA_TRANSFER,
		.pause = 0,
		.finish_intr = 1,
};

static struct spi_board_info spi_silead_board_info[] __initdata = {
  [0] = {
	    .modalias = "silead_fp",
		.max_speed_hz = 5000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data= &chip_config 
   },
};
#endif

//static int spidev_setup_eint(struct spi_device *spi);		
/*-------------------------------------------------------------------------*/
#if defined(SL_USE_PLATFORM_BUS)
static int spidev_probe(struct platform_device *spi)
#elif defined(SL_USE_SPI_BUS)
static int spidev_probe(struct spi_device *spi)
#endif
{
    struct spidev_data	*spidev;
    int			status;
#ifdef sileadDBG
    unsigned int val;
    char tmp[12];
#endif
    unsigned long		minor;
    dev_t devno;
    struct kobject *power_kobj;
    #if 0
    int fp_vendor_tee;

    //add by yinglong.tang
    fp_vendor_tee = get_fp_vendor();
    SL_LOGD("get_fp_vendor =  %d\n", fp_vendor_tee);
    if (fp_vendor_tee != SILEAD_VENDOR) {
        SL_LOGD("%s, Fingerprint vendor not silead return!\n", __func__);
        return -1;	
    }
    #endif
    //add by yinglong.tang

    #if 0
    //Read fp id for silead. add by yinglong.tang
    if (read_fpId_pin_value(&spi->dev, "fpid-gpio") == __HIGH /*HIGH*/) {
        printk("read_fpId_pin_value === _HIGH");
    } else {
        printk("read_fpId_pin_value != _HIGH,not silead ic return.");
        return -1;
    }
    #endif

    int fp_vendor_tee;
    fp_vendor_tee = get_fp_vendor();
    SL_LOGD("get_fp_vendor =  %d\n", fp_vendor_tee);
    if (fp_vendor_tee == FP_VENDOR_INVALID) {
        if (read_fpId_pin_value(&spi->dev, "fpid-gpio") == __HIGH /*HIGH*/) {
            printk("read_fpId_pin_value === _HIGH");
        } else {
            printk("read_fpId_pin_value != _HIGH,not silead ic return.");
            return -1;
        }	
    } else {
        printk("fp_vendor != silead,not silead ic return.");
        return -1;
    }
    
    /* Claim our 256 reserved device numbers.  Then register a class
    * that will key udev/mdev to add/remove /dev nodes.  Last, register
    * the driver which manages those device numbers.
    */
    BUILD_BUG_ON(N_SPI_MINORS > 256);

    status = alloc_chrdev_region(&devno, 0,255, "sileadfp");
    if(status <0 ) {
        SL_LOGD("alloc_chrdev_region error\n");
        return status;
    }

    spidev_major = MAJOR(devno);
    cdev_init(&spicdev, &spidev_fops);
    spicdev.owner = THIS_MODULE;
    status = cdev_add(&spicdev,MKDEV(spidev_major, 0),N_SPI_MINORS);
    if(status != 0) {
        SL_LOGD("cdev_add error\n");
        return status;
    }

    spidev_class = class_create(THIS_MODULE, "spidev");
    if (IS_ERR(spidev_class)) {
        SL_LOGD("class_create error\n");
        unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
        status =  PTR_ERR(spidev_class);
        return status;
    }
    
    /* Allocate driver data */
    spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
    if (!spidev)
    {
        class_destroy(spidev_class);
        unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
        return -ENOMEM;
    }
 
    SL_LOGD("spidev_probe\n");  
    /* Initialize the driver data */
    fp_spidev = spidev;
    spidev->spi = spi;
//	spi->mode = SPI_MODE_0;
//	spi->bits_per_word = 8;
    spin_lock_init(&spidev->spi_lock);
    mutex_init(&spidev->buf_lock);

    INIT_LIST_HEAD(&spidev->device_entry);
    INIT_WORK(&spidev->int_work,spidev_irq_work);
    wake_lock_init(&spidev->wake_lock, WAKE_LOCK_SUSPEND, "silead_wake_lock");
    spidev->wqueue = create_singlethread_workqueue("silead_wq");

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    SL_LOGD("minor:%ld\n",minor);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        spidev->devt = MKDEV(spidev_major, minor);
        dev = device_create(spidev_class, &spi->dev, spidev->devt,
                            spidev, "silead_fp_dev");

        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
        SL_LOGD("status = %d\n", status);
    } else {
        dev_dbg(&spi->dev, "no minor number available!\n");
        status = -ENODEV;
		
    }
    if (status == 0) {
        set_bit(minor, minors);
        list_add(&spidev->device_entry, &device_list);
		
    }
    mutex_unlock(&device_list_lock);

    silead_power_init(spidev);
    silead_power_ctl(spidev, 1);
    silead_parse_dt(&spi->dev, spidev);
    silead_gpio_config(spidev);

    /* Init Poll Wait */
    init_waitqueue_head(&silead_poll_wq);

//add by bacon for fp_wake_lock
    power_kobj = kobject_create_and_add("silead", NULL);
    if (power_kobj == NULL)
    {
        cdev_del(&spicdev);
        class_destroy(spidev_class);
        unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
        kfree(spidev);
        fp_spidev = NULL;
        SL_LOGD("%s kobject_create_and_add error!\n", __func__);
        return -ENOMEM;
    }

    status = sysfs_create_group(power_kobj, &attr_group);
    if (status)
    {
        cdev_del(&spicdev);
        class_destroy(spidev_class);
        unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
        kobject_del(power_kobj);
        kfree(spidev);
        fp_spidev = NULL;
        SL_LOGD("%s sysfs_create_group error!\n", __func__);
        return status;
    }
//add by bacon for fp_wake_lock

    if (status == 0){
#if defined(SL_USE_PLATFORM_BUS)
    		platform_set_drvdata(spi, spidev);
#elif defined(SL_USE_SPI_BUS)
    		spi_set_drvdata(spi, spidev);
#endif
	
#ifdef sileadDBG
    		spidev_reset_hw(spidev);
    		val=spidev_read_reg(spidev, 0xfc);//remove for TOS in 20160706
    		intToStr(val,tmp);
    		tmp[8]='\0';
    		SL_LOGD(" [%s] the chip id = %s \n",__func__,tmp);	
#endif
		}
    else{
    		cdev_del(&spicdev);
    		class_destroy(spidev_class);
    		unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
    		kobject_del(power_kobj);
    		kfree(spidev);
    		fp_spidev = NULL;
		}

		full_fp_chip_name(SILEAD_FP_NAME);

    return status;
}

#if defined(SL_USE_PLATFORM_BUS)
static int spidev_remove(struct platform_device *spi)
#elif defined(SL_USE_SPI_BUS)
static int spidev_remove(struct spi_device *spi)
#endif
{
#if defined(SL_USE_PLATFORM_BUS)
    struct spidev_data	*spidev = platform_get_drvdata(spi);
#elif defined(SL_USE_SPI_BUS)
    struct spidev_data	*spidev = spi_get_drvdata(spi);
#endif

    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&spidev->spi_lock);
    spidev->spi = NULL;
#if defined(SL_USE_PLATFORM_BUS)
    platform_set_drvdata(spi, NULL);
#elif defined(SL_USE_SPI_BUS)
    spi_set_drvdata(spi, NULL);
#endif
    spin_unlock_irq(&spidev->spi_lock);

    if (spidev->int_irq)
    {
        free_irq(spidev->int_irq, spidev);
        irq_enabled = 0;
        irq_requested = 0;
        irq_counter = 0;
    }
    silead_power_deinit(spidev);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&spidev->device_entry);
    device_destroy(spidev_class, spidev->devt);
    clear_bit(MINOR(spidev->devt), minors);
    wake_lock_destroy(&spidev->wake_lock);
    mutex_unlock(&device_list_lock);

    return 0;
}

#if 0
static int spidev_suspend(struct spi_device *spi, pm_message_t mesg)
{
    return 0;
}
static int spidev_resume(struct spi_device *spi)
{
    return 0;
}
#endif

static int __init spidev_init(void)
{
    int status; 

#if defined(SL_USE_PLATFORM_BUS)
    status = platform_driver_register(&silead_fp_driver);
#elif defined(SL_USE_SPI_BUS)
    spi_register_board_info(spi_silead_board_info,ARRAY_SIZE(spi_silead_board_info));
    status = spi_register_driver(&spidev_spi_driver);
#endif

    if (status < 0) {
        SL_LOGD("%s register SPI driver error!\n", __func__);
    }

    return status;
}

static void __exit spidev_exit(void)
{
    if(fp_spidev != NULL)
    {
        cdev_del(&spicdev);
    }
#if defined(SL_USE_PLATFORM_BUS)
    platform_driver_unregister(&silead_fp_driver);
#elif defined(SL_USE_SPI_BUS)
    spi_unregister_driver(&spidev_spi_driver);
#endif
    if(fp_spidev != NULL)
    {
        class_destroy(spidev_class);
        unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
        kfree(fp_spidev);
        fp_spidev = NULL;
    }
}

module_init(spidev_init);
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
