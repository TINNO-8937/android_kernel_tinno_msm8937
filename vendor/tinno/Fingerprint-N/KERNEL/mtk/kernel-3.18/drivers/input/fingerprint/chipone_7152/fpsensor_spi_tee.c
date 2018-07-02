#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/gpio.h>

#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/wakelock.h>

/* MTK header */
#include "mt_spi.h"
#include "mt_spi_hal.h"
#include "mt_gpio.h"
#include "mach/gpio_const.h"

#include "fpsensor_spi_tee.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#include "../fp_drv/fp_drv.h"

/* For Tinno fp_drv to get vendor */
#define CHIPONE_DEV_INFO "chipone_fp"

/*device name*/
#define FPSENSOR_DEV_NAME       "chipone_fp"
#define FPSENSOR_CLASS_NAME     "chipone_fp"
#define FPSENSOR_MAJOR          0
#define N_SPI_MINORS            32    /* ... up to 256 */

#define FPSENSOR_SPI_VERSION    "fpsensor_spi_tee_v0.1"
#define FPSENSOR_INPUT_NAME     "fpsensor_keys"

/**************************feature control******************************/
#define FPSENSOR_IOCTL    1
#define FPSENSOR_SYSFS    0
#define FPSENSOR_INPUT    0
/***********************input *************************/
#ifndef FPSENSOR_INPUT_HOME_KEY
/* on MTK EVB board, home key has been redefine to KEY_HOMEPAGE! */
/* double check the define on customer board!!! */
#define FPSENSOR_INPUT_HOME_KEY     KEY_HOMEPAGE /* KEY_HOME */
#define FPSENSOR_INPUT_MENU_KEY     KEY_MENU
#define FPSENSOR_INPUT_BACK_KEY     KEY_BACK
#define FPSENSOR_INPUT_FF_KEY       KEY_POWER
#define FPSENSOR_INPUT_CAMERA_KEY   KEY_CAMERA
#define FPSENSOR_INPUT_OTHER_KEY    KEY_VOLUMEDOWN  /* temporary key value for capture use */
#endif

#define FPSENSOR_NAV_UP_KEY     19//KEY_UP
#define FPSENSOR_NAV_DOWN_KEY   20//KEY_DOWN
#define FPSENSOR_NAV_LEFT_KEY   21//KEY_LEFT
#define FPSENSOR_NAV_RIGHT_KEY  22//KEY_RIGHT
#define FPSENSOR_NAV_TAP_KEY    23
/***********************GPIO setting port layer*************************/
/* customer hardware port layer, please change according to customer's hardware */
#define GPIO_PIN_IRQ   86

/*************************************************************/
static struct wake_lock fpsensor_timeout_wakelock;

fpsensor_data_t *g_fpsensor = NULL;

static struct fasync_struct *fasync_queue = NULL;
static uint32_t ftm_finger_status = 0; // 0x01 down 0x02 up

static char fpsensor_ver_buf[FPSENSOR_MAX_VER_BUF_LEN] = {0x00};
static char fpsensor_chip_name[FPSENSOR_MAX_CHIP_NAME_LEN] = {0};

#define ROUND_UP(x, align)        ((x+(align-1))&~(align-1))

/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)

/* debug log setting */
static u8 fpsensor_debug_level = ERR_LOG;

#define fpsensor_debug(level, fmt, args...) do { \
        if (fpsensor_debug_level >= level) {\
            printk( "[fpsensor] " fmt, ##args); \
        } \
    } while (0)

#define FUNC_ENTRY()  fpsensor_debug(DEBUG_LOG, "%s, %d, entry\n", __func__, __LINE__)
#define FUNC_EXIT()  fpsensor_debug(DEBUG_LOG, "%s, %d, exit\n", __func__, __LINE__)


#define FPSENSOR_SPI_BUS_DYNAMIC 1
#if FPSENSOR_SPI_BUS_DYNAMIC
static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias = FPSENSOR_DEV_NAME,
        .bus_num = 0,
        .chip_select = 0,        
        .mode = SPI_MODE_0,
       // .controller_data = &fpsensor_spi_conf_mt65xx, //&spi_conf
       },
};
#endif



/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration                              */
/* -------------------------------------------------------------------- */
static DEFINE_MUTEX(spidev_set_gpio_mutex);
static void spidev_gpio_as_int(fpsensor_data_t *fpsensor)
{
    fpsensor_trace( "[fpsensor]----%s---\n", __func__);
    mutex_lock(&spidev_set_gpio_mutex);
    //printk("[fpsensor]spidev_gpio_as_int\n");
    pinctrl_select_state(fpsensor->pinctrl1, fpsensor->eint_as_int);
    mutex_unlock(&spidev_set_gpio_mutex);
}
void fpsensor_gpio_output_dts(int gpio, int level)
{
    mutex_lock(&spidev_set_gpio_mutex);
    //printk("[fpsensor]fpsensor_gpio_output_dts: gpio= %d, level = %d\n", gpio, level);
    if (gpio == FPSENSOR_RST_PIN)
    {
        if (level)
        {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_high);
        }
        else
        {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_low);
        }
    }

    mutex_unlock(&spidev_set_gpio_mutex);
} 


int fpsensor_gpio_wirte(int gpio, int value)
{
    fpsensor_gpio_output_dts(gpio, value);
    return 0;
}
int fpsensor_gpio_read(int gpio)
{
    return gpio_get_value(gpio);
}

int fpsensor_spidev_dts_init(fpsensor_data_t *fpsensor)
{
    struct device_node *node;
    int ret = 0;
    fpsensor_printk( "%s\n", __func__);
    node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
    if (node)
    {
        fpsensor->fp_rst_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fp_rst_low");
        if (IS_ERR(fpsensor->fp_rst_low))
        {
            ret = PTR_ERR(fpsensor->fp_rst_low);
            fpsensor_error("fpensor Cannot find fp pinctrl fp_rst_low!\n");
            return ret;
        }
        fpsensor->fp_rst_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fp_rst_high");
        if (IS_ERR(fpsensor->fp_rst_high))
        {
            ret = PTR_ERR(fpsensor->fp_rst_high);
            fpsensor_error( "fpsensor Cannot find fp pinctrl fp_rst_high!\n");
            return ret;
        }

        fpsensor->eint_as_int = pinctrl_lookup_state(fpsensor->pinctrl1, "eint_in_low");
        if (IS_ERR(fpsensor->eint_as_int))
        {
            ret = PTR_ERR(fpsensor->eint_as_int);
            fpsensor_error( "fpsensor Cannot find fp pinctrl eint_as_int!\n");
            return ret;
        }
        fpsensor->eint_in_low = pinctrl_lookup_state(fpsensor->pinctrl1, "eint_in_low");
        if (IS_ERR(fpsensor->eint_as_int))
        {
            ret = PTR_ERR(fpsensor->eint_as_int);
            fpsensor_error( "fpsensor Cannot find fp pinctrl eint_as_int!\n");
            return ret;
        }
        fpsensor->eint_in_high= pinctrl_lookup_state(fpsensor->pinctrl1, "eint_in_high");
    	if (IS_ERR(fpsensor->eint_in_high)) {
    		ret = PTR_ERR(fpsensor->eint_in_high);
    		fpsensor_error( " Cannot find fp pinctrl eint_in_high!\n");
    		return ret;
    	}
    
    	fpsensor->eint_in_float = pinctrl_lookup_state(fpsensor->pinctrl1, "eint_in_float");
    	if (IS_ERR(fpsensor->eint_in_float)) {
    		ret = PTR_ERR(fpsensor->eint_in_float);
    		fpsensor_error( " Cannot find fp pinctrl eint_in_float!\n");
    		return ret;
    	}
    }
    else
    {
        fpsensor_error("fpensor Cannot find node!\n");
    }
    return 0;
}
/* delay us after reset */
static void fpsensor_hw_reset(int delay)
{
    FUNC_ENTRY();

    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,    1);
    udelay(100);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  0);
    udelay(1000);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  1);
    if (delay)
    {
        /* delay is configurable */
        udelay(delay);
    }

    FUNC_EXIT();
    return;
}

static void fpsensor_spi_clk_enable(u8 bonoff)
{
	static int count = 0;

#ifdef CONFIG_MTK_CLKMGR
    if (bonoff && (count == 0)) {
        fpsensor_debug(DEBUG_LOG, "%s, start to enable spi clk && count = %d.\n", __func__, count);
        enable_clock(MT_CG_PERI_SPI0, "spi");
        count = 1;
    } else if ((count > 0) && (bonoff == 0)) {
        fpsensor_debug(DEBUG_LOG, "%s, start to disable spi clk&& count = %d.\n", __func__, count);
        disable_clock(MT_CG_PERI_SPI0, "spi");
        count = 0;
    }
#else
	/* changed after MT6797 platform */
	struct mt_spi_t *ms = NULL;

	ms = spi_master_get_devdata(g_fpsensor->spi->master);

	if (bonoff && (count == 0)) {
		mt_spi_enable_clk(ms);    // FOR MT6797
//		clk_enable(ms->clk_main); // FOR MT6755/MT6750
		count = 1;
	} else if ((count > 0) && (bonoff == 0)) {
		mt_spi_disable_clk(ms);    // FOR MT6797
//		clk_disable(ms->clk_main); // FOR MT6755/MT6750
		count = 0;
	}
#endif
}
// static void fpsensor_hw_power_enable(u8 onoff)
// {
//     static int enable = 1;
//     if (onoff && enable)
//     {
//         pinctrl_select_state(g_fpsensor->pinctrl_gpios, g_fpsensor->pins_power_on);
//         enable = 0;
//     }
//     else if (!onoff && !enable)
//     {
//         pinctrl_select_state(g_fpsensor->pinctrl_gpios, g_fpsensor->pins_power_off);
//         enable = 1;
//     }
// }

static int fpsensor_irq_gpio_cfg(void)
{
    int error = 0;
    struct device_node *node;
    fpsensor_data_t *fpsensor;
    u32 ints[2] = {0, 0};
    fpsensor_printk("%s\n", __func__);

    fpsensor = g_fpsensor;

    spidev_gpio_as_int(fpsensor);

    node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
    if ( node)
    {
        of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_request(ints[0], "fpsensor-irq");
        //gpio_set_debounce(ints[0], ints[1]);
        fpsensor_printk("[fpsensor]ints[0] = %d,is irq_gpio , ints[1] = %d!!\n", ints[0], ints[1]);
        fpsensor->irq_gpio = ints[0];
        fpsensor->irq = irq_of_parse_and_map(node, 0);  // get irq number
        if (!fpsensor->irq)
        {
            printk("fpsensor irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        fpsensor_printk(" [fpsensor]fpsensor->irq= %d,fpsensor>irq_gpio = %d\n", fpsensor->irq,
                        fpsensor->irq_gpio);
    }
    else
    {
        printk("fpsensor null irq node!!\n");
        return -EINVAL;
    }

    return error;

}
static void fpsensor_enable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();
    setRcvIRQ(0);
    if (0 == fpsensor_dev->device_available)
    {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
    }
    else
    {
        if (1 == fpsensor_dev->irq_count)
        {
            fpsensor_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
        }
        else
        {
            enable_irq(fpsensor_dev->irq);
            fpsensor_dev->irq_count = 1;
            fpsensor_debug(INFO_LOG, "%s enable interrupt!\n", __func__);
        }
    }
    FUNC_EXIT();
    return;
}

static void fpsensor_disable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();

    if (0 == fpsensor_dev->device_available)
    {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
    }
    else
    {
        if (0 == fpsensor_dev->irq_count)
        {
            fpsensor_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
        }
        else
        {
            disable_irq(fpsensor_dev->irq);
            fpsensor_dev->irq_count = 0;
            fpsensor_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
        }
    }
    setRcvIRQ(0);
    FUNC_EXIT();
    return;
}

/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */

static ssize_t fpsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support read opertion in TEE version\n");
    return -EFAULT;
}

static ssize_t fpsensor_write(struct file *filp, const char __user *buf,
                              size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support write opertion in TEE version\n");
    return -EFAULT;
}

static irqreturn_t fpsensor_irq(int irq, void *handle)
{

    fpsensor_data_t *fpsensor_dev = (fpsensor_data_t *)handle;

    wake_lock_timeout(&fpsensor_timeout_wakelock, msecs_to_jiffies(3000));
#if FPSENSOR_IOCTL
    setRcvIRQ(1);
#endif
    wake_up_interruptible(&fpsensor_dev->wq_irq_return);
    fpsensor_dev->sig_count++;

    return IRQ_HANDLED;
}

void setRcvIRQ(int val)
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    // fpsensor_debug(INFO_LOG, "[rickon]: %s befor val :  %d ; set val : %d   \n", __func__, fpsensor_dev-> RcvIRQ, val);
    fpsensor_dev-> RcvIRQ = val;
}


//#include <linux/dev_info.h>
//static struct devinfo_struct *s_DEVINFO_fpsensor;
static long fpsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    fpsensor_data_t *fpsensor_dev = NULL;
    struct fpsensor_key fpsensor_key;
#if FPSENSOR_INPUT
    uint32_t key_event;
#endif
    int retval = 0;
    unsigned int val = 0;

    FUNC_ENTRY();
    fpsensor_debug(INFO_LOG, "[rickon]: fpsensor ioctl cmd : 0x%x \n", cmd );
    fpsensor_dev = (fpsensor_data_t *)filp->private_data;
    //clear cancel flag
    fpsensor_dev->cancel = 0 ;
    switch (cmd)
    {
        case FPSENSOR_IOC_INIT:
            fpsensor_debug(INFO_LOG, "%s: fpsensor init started======\n", __func__);
            fpsensor_irq_gpio_cfg();
            retval = request_threaded_irq(fpsensor_dev->irq, fpsensor_irq, NULL,
                                          IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&(fpsensor_dev->spi->dev)), fpsensor_dev);
            if (retval == 0)
            {
                fpsensor_debug(ERR_LOG, " irq thread reqquest success!\n");
            }
            else
            {
                fpsensor_debug(ERR_LOG, " irq thread request failed , retval =%d \n", retval);
            }
            fpsensor_dev->device_available = 1;
            fpsensor_dev->irq_count = 1;
            fpsensor_disable_irq(fpsensor_dev);

            fpsensor_dev->sig_count = 0;
			/*
            s_DEVINFO_fpsensor = (struct devinfo_struct *) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
            s_DEVINFO_fpsensor->device_type = "FP";
            s_DEVINFO_fpsensor->device_module = "oflim";
            s_DEVINFO_fpsensor->device_vendor = "Chipone";
            s_DEVINFO_fpsensor->device_ic = "ICNT7152";
            s_DEVINFO_fpsensor->device_version = "NULL";
            s_DEVINFO_fpsensor->device_info = "fingerprint";
            s_DEVINFO_fpsensor->device_used = DEVINFO_USED;
            devinfo_check_add_device(s_DEVINFO_fpsensor);
			*/
            wake_lock_init(&fpsensor_timeout_wakelock, WAKE_LOCK_SUSPEND, "fpsensor timeout wakelock");
            fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);
            break;

        case FPSENSOR_IOC_EXIT:
            fpsensor_disable_irq(fpsensor_dev);
            if (fpsensor_dev->irq)
            {
                free_irq(fpsensor_dev->irq, fpsensor_dev);
                fpsensor_dev->irq_count = 0;
            }
            fpsensor_dev->device_available = 0;
            fpsensor_debug(INFO_LOG, "%s: fpsensor exit finished======\n", __func__);
            break;

        case FPSENSOR_IOC_RESET:
            fpsensor_debug(INFO_LOG, "%s: chip reset command\n", __func__);
            fpsensor_hw_reset(1250);
            break;

        case FPSENSOR_IOC_ENABLE_IRQ:
            fpsensor_debug(INFO_LOG, "%s: chip ENable IRQ command\n", __func__);
            fpsensor_enable_irq(fpsensor_dev);
            break;

        case FPSENSOR_IOC_DISABLE_IRQ:
            fpsensor_debug(INFO_LOG, "%s: chip disable IRQ command\n", __func__);
            fpsensor_disable_irq(fpsensor_dev);
            break;
        case FPSENSOR_IOC_GET_INT_VAL:
            val = __gpio_get_value(GPIO_PIN_IRQ);
            if (copy_to_user((void __user *)arg, (void *)&val, sizeof(unsigned int)))
            {
                fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
                retval = -EFAULT;
                break;
            }
            retval = 0;
            break;
        case FPSENSOR_IOC_ENABLE_SPI_CLK:
            fpsensor_debug(INFO_LOG, "%s: ENABLE_SPI_CLK ======\n", __func__);
            fpsensor_spi_clk_enable(1);
            break;
        case FPSENSOR_IOC_DISABLE_SPI_CLK:
            fpsensor_debug(INFO_LOG, "%s: DISABLE_SPI_CLK ======\n", __func__);
            fpsensor_spi_clk_enable(0);
            break;

        case FPSENSOR_IOC_ENABLE_POWER:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_POWER ======\n", __func__);
           // fpsensor_hw_power_enable(1);
            break;

        case FPSENSOR_IOC_DISABLE_POWER:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_DISABLE_POWER ======\n", __func__);
           // fpsensor_hw_power_enable(0);
            break;


        case FPSENSOR_IOC_INPUT_KEY_EVENT:
            if (copy_from_user(&fpsensor_key, (struct fpsensor_key *)arg, sizeof(struct fpsensor_key)))
            {
                fpsensor_debug(ERR_LOG, "Failed to copy input key event from user to kernel\n");
                retval = -EFAULT;
                break;
            }
#if FPSENSOR_INPUT
            if (FPSENSOR_KEY_HOME == fpsensor_key.key)
            {
                key_event = FPSENSOR_INPUT_HOME_KEY;
            }
            else if (FPSENSOR_KEY_POWER == fpsensor_key.key)
            {
                key_event = FPSENSOR_INPUT_FF_KEY;
            }
            else if (FPSENSOR_KEY_CAPTURE == fpsensor_key.key)
            {
                key_event = FPSENSOR_INPUT_CAMERA_KEY;
            }
            else
            {
                /* add special key define */
                key_event = FPSENSOR_INPUT_OTHER_KEY;
            }
            fpsensor_debug(INFO_LOG, "%s: received key event[%d], key=%d, value=%d\n",
                           __func__, key_event, fpsensor_key.key, fpsensor_key.value);
            if ((FPSENSOR_KEY_POWER == fpsensor_key.key || FPSENSOR_KEY_CAPTURE == fpsensor_key.key)
                && (fpsensor_key.value == 1))
            {
                input_report_key(fpsensor_dev->input, key_event, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, key_event, 0);
                input_sync(fpsensor_dev->input);
            }
            else if (FPSENSOR_KEY_UP == fpsensor_key.key)
            {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_UP_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_UP_KEY, 0);
                input_sync(fpsensor_dev->input);
            }
            else if (FPSENSOR_KEY_DOWN == fpsensor_key.key)
            {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_DOWN_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_DOWN_KEY, 0);
                input_sync(fpsensor_dev->input);
            }
            else if (FPSENSOR_KEY_RIGHT == fpsensor_key.key)
            {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_RIGHT_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_RIGHT_KEY, 0);
                input_sync(fpsensor_dev->input);
            }
            else if (FPSENSOR_KEY_LEFT == fpsensor_key.key)
            {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_LEFT_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_LEFT_KEY, 0);
                input_sync(fpsensor_dev->input);
            }
            else  if (FPSENSOR_KEY_TAP == fpsensor_key.key)
            {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_TAP_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_TAP_KEY, 0);
                input_sync(fpsensor_dev->input);
            }
            else if ((FPSENSOR_KEY_POWER != fpsensor_key.key) && (FPSENSOR_KEY_CAPTURE != fpsensor_key.key))
            {
                input_report_key(fpsensor_dev->input, key_event, fpsensor_key.value);
                input_sync(fpsensor_dev->input);
            }
#endif
            break;

        case FPSENSOR_IOC_ENTER_SLEEP_MODE:
            fpsensor_dev->is_sleep_mode = 1;
            break;
        case FPSENSOR_IOC_REMOVE:
#if FPSENSOR_INPUT
            if (fpsensor_dev->input != NULL)
            {
                input_unregister_device(fpsensor_dev->input);
            }
#endif
            if (fpsensor_dev->device != NULL)
            {
                device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
            }
            if (fpsensor_dev->class != NULL )
            {
                unregister_chrdev_region(fpsensor_dev->devno, 1);
                class_destroy(fpsensor_dev->class);
            }
            if (fpsensor_dev->users == 0)
            {
#if FPSENSOR_INPUT
                if (fpsensor_dev->input != NULL)
                {
                    input_unregister_device(fpsensor_dev->input);
                }
#endif
                // if (fpsensor_dev->buffer != NULL)
                // kfree(fpsensor_dev->buffer);

                kfree(fpsensor_dev);
            }
            //mutex_unlock(&device_list_lock);
			/*
            if (s_DEVINFO_fpsensor != NULL)
            {
                s_DEVINFO_fpsensor->device_type = "FP";
                s_DEVINFO_fpsensor->device_module = "oflim";
                s_DEVINFO_fpsensor->device_vendor = "NULL";
                s_DEVINFO_fpsensor->device_ic = "LC1550";
                s_DEVINFO_fpsensor->device_version = "NULL";
                s_DEVINFO_fpsensor->device_info = "fingerprint";
                s_DEVINFO_fpsensor->device_used = DEVINFO_UNUSED;
                devinfo_check_add_device(s_DEVINFO_fpsensor);
            }
			*/
           // fpsensor_hw_power_enable(0);
            fpsensor_spi_clk_enable(0);
            fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
            break;

        case FPSENSOR_IOC_CANCEL_WAIT:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR CANCEL WAIT\n", __func__);
            fpsensor_dev->cancel = 1;
            wake_up_interruptible(&fpsensor_dev->wq_irq_return);
            break;

        case FPSENSOR_IOC_SET_DEV_INFO:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR SET CHIP NAME\n", __func__);
            if (copy_from_user(fpsensor_chip_name, (char __user*)arg, FPSENSOR_MAX_CHIP_NAME_LEN)) {
                fpsensor_debug(ERR_LOG, "fpsensor Failed to copy chip name from user\n");
                retval = -EFAULT;
                break;
            }
            printk(KERN_INFO "%s: chip name: %s\n", __func__, fpsensor_chip_name);
            full_fp_chip_name(fpsensor_chip_name);
            break;
        case FPSENSOR_IOC_FTM_SET_FINGER_STATE:
            ftm_finger_status = 0;
            if (copy_from_user(&ftm_finger_status, (u32 __user*)arg, sizeof(u32))) {
                fpsensor_debug(ERR_LOG, "fpsensor Failed to copy data from user\n");
                retval = -EFAULT;
                break;
            }
            printk(KERN_INFO "%s: FPSENSOR_IOC_FTM_SET_FINGER_STATE: %d\n", __func__, ftm_finger_status);
            if(fasync_queue) {
				kill_fasync(&fasync_queue, SIGIO, POLL_IN);
				printk("%s: kill_fasync to user(ftm_finger_status) !\n", __func__);
			}
            break;
        case FPSENSOR_IOC_FTM_GET_FINGER_STATE:
            if (copy_to_user((u32 __user*)arg, &ftm_finger_status, sizeof(u32))) {
                fpsensor_debug(ERR_LOG, "fpsensor Failed to copy data to user\n");
                retval = -EFAULT;
                break;
            }
			printk("%s: __put_user(ftm_finger_status)\n", __func__);
            break;
        case FPSENSOR_IOC_RELEASE_VERSION:
            if (copy_from_user(fpsensor_ver_buf, (char __user*)arg, FPSENSOR_MAX_VER_BUF_LEN)) {
                fpsensor_debug(ERR_LOG, "fpsensor Failed to copy version from user\n");
                retval = -EFAULT;
                break;
            }
            printk(KERN_INFO "%s: release version info: %s\n", __func__, fpsensor_ver_buf);
            full_fp_chip_info(fpsensor_ver_buf);
            break;

        default:
            fpsensor_debug(ERR_LOG, "fpsensor doesn't support this command(%d)\n", cmd);
            break;
    }

    FUNC_EXIT();
    return retval;
}
static long fpsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fpsensor_ioctl(filp, cmd, (unsigned long)(arg));
}

static unsigned int fpsensor_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int ret = 0;
    fpsensor_debug(ERR_LOG, " support poll opertion  in   version\n");
    ret |= POLLIN;
    poll_wait(filp, &g_fpsensor->wq_irq_return, wait);
    if (g_fpsensor->cancel == 1 )
    {
        fpsensor_debug(ERR_LOG, " cancle\n");
        ret =  POLLERR;
        g_fpsensor->cancel = 0;
        return ret;
    }
    if ( g_fpsensor->RcvIRQ)
    {
        fpsensor_debug(ERR_LOG, " get irq\n");
        ret |= POLLRDNORM;
    }
    else
    {
        ret = 0;
    }
    return ret;
}


/* -------------------------------------------------------------------- */
/* device function                                                      */
/* -------------------------------------------------------------------- */
static int fpsensor_open(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;

    FUNC_ENTRY();
    fpsensor_dev = container_of(inode->i_cdev, fpsensor_data_t, cdev);
    fpsensor_dev->users++;
    fpsensor_dev->device_available = 1;
    filp->private_data = fpsensor_dev;
    FUNC_EXIT();
    return 0;
}

static int fpsensor_release(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;
    int    status = 0;

    FUNC_ENTRY();
    fpsensor_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    fpsensor_dev->users--;
    if (!fpsensor_dev->users)
    {
        fpsensor_debug(INFO_LOG, "%s, disble_irq. irq = %d\n", __func__, fpsensor_dev->irq);
        fpsensor_disable_irq(fpsensor_dev);
    }
    fpsensor_dev->device_available = 0;
    FUNC_EXIT();
    return status;
}

static int fpsensor_fasync(int fd, struct file * filp, int on)
{
	printk("%s enter \n",__func__);
	return fasync_helper(fd, filp, on, &fasync_queue);
}

static const struct file_operations fpsensor_fops =
{
    .owner =    THIS_MODULE,

    .write =    fpsensor_write,
    .read =        fpsensor_read,
    .unlocked_ioctl = fpsensor_ioctl,
    .compat_ioctl   = fpsensor_compat_ioctl,
    .open =        fpsensor_open,
    .release =    fpsensor_release,
    .poll    = fpsensor_poll,
    .fasync         = fpsensor_fasync,
};

static int fpsensor_create_class(fpsensor_data_t *fpsensor)
{
    int error = 0;

    fpsensor->class = class_create(THIS_MODULE, FPSENSOR_CLASS_NAME);
    if (IS_ERR(fpsensor->class))
    {
        fpsensor_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
        error = PTR_ERR(fpsensor->class);
    }

    return error;
}

static int fpsensor_create_device(fpsensor_data_t *fpsensor)
{
    int error = 0;


    if (FPSENSOR_MAJOR > 0)
    {
        //fpsensor->devno = MKDEV(FPSENSOR_MAJOR, fpsensor_device_count++);
        //error = register_chrdev_region(fpsensor->devno,
        //                 1,
        //                 FPSENSOR_DEV_NAME);
    }
    else
    {
        error = alloc_chrdev_region(&fpsensor->devno,
                                    fpsensor->device_count++,
                                    1,
                                    FPSENSOR_DEV_NAME);
    }

    if (error < 0)
    {
        fpsensor_debug(ERR_LOG,
                       "%s: FAILED %d.\n", __func__, error);
        goto out;

    }
    else
    {
        fpsensor_debug(INFO_LOG, "%s: major=%d, minor=%d\n",
                       __func__,
                       MAJOR(fpsensor->devno),
                       MINOR(fpsensor->devno));
    }

    fpsensor->device = device_create(fpsensor->class, &(fpsensor->spi->dev), fpsensor->devno,
                                     fpsensor, FPSENSOR_DEV_NAME);

    if (IS_ERR(fpsensor->device))
    {
        fpsensor_debug(ERR_LOG, "device_create failed.\n");
        error = PTR_ERR(fpsensor->device);
    }
out:
    return error;
}

extern int get_fp_vendor(void);
enum {
    FP_VENDOR_INVALID = 0,
    FPC_VENDOR,
    ELAN_VENDOR,
    GOODIX_VENDOR,
    CHIPONE_VENDOR
};

#if defined(USE_SPI_BUS)
static int fpsensor_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int fpsensor_probe(struct platform_device *spi)
#endif
{
    struct device *dev = &spi->dev;
    fpsensor_data_t *fpsensor_dev = NULL;
    int error = 0;
    // u16 i = 0;
    // unsigned long minor;f
    int status = -EINVAL;
	int fp_vendor_tee;

    FUNC_ENTRY();

    /* Check whether is chipone chip */
	fp_vendor_tee = get_fp_vendor();
	fpsensor_debug(ERR_LOG, "get_fp_vendor =  %d\n", fp_vendor_tee);
	if (fp_vendor_tee != CHIPONE_VENDOR) {
		fpsensor_debug(ERR_LOG, "%s, Fingerprint vendor not chipone return!\n", __func__);
		return -1;	
	}

    /* Allocate driver data */
    fpsensor_dev = kzalloc(sizeof(*fpsensor_dev), GFP_KERNEL);
    if (!fpsensor_dev)
    {
        fpsensor_debug(ERR_LOG, "%s, Failed to alloc memory for fpsensor device.\n", __func__);
        FUNC_EXIT();
        return -ENOMEM;
    }
    fpsensor_dev->device = dev ;

    g_fpsensor = fpsensor_dev;
    /* Initialize the driver data */
    mutex_init(&fpsensor_dev->buf_lock);

#if defined(USE_SPI_BUS)
    spi_set_drvdata(spi, fpsensor_dev);
#elif defined(USE_PLATFORM_BUS)
    platform_set_drvdata(spi, fpsensor_dev);
#endif
    fpsensor_dev->spi = spi;
    // INIT_LIST_HEAD(&fpsensor_dev->device_entry);
    fpsensor_dev->device_available = 0;
    fpsensor_dev->irq = 0;
    fpsensor_dev->probe_finish = 0;
    fpsensor_dev->device_count     = 0;
    fpsensor_dev->users = 0;
    /*setup fpsensor configurations.*/
    fpsensor_debug(INFO_LOG, "%s, Setting fpsensor device configuration.\n", __func__);
    // fpsensor_irq_gpio_cfg();
    // fpsensor_reset_gpio_cfg();
    // dts read
    spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
    fpsensor_dev->pinctrl1 = devm_pinctrl_get(&spi->dev);
    if (IS_ERR(fpsensor_dev->pinctrl1))
    {
        error = PTR_ERR(fpsensor_dev->pinctrl1);
        fpsensor_error("fpsensor Cannot find fp pinctrl1.\n");
        goto err1;
    }
    fpsensor_spidev_dts_init(fpsensor_dev);
    error = fpsensor_create_class(fpsensor_dev);
    if (error)
    {
        goto err2;
    }
    error = fpsensor_create_device(fpsensor_dev);
    if (error)
    {
        goto err2;
    }
    cdev_init(&fpsensor_dev->cdev, &fpsensor_fops);
    fpsensor_dev->cdev.owner = THIS_MODULE;
    error = cdev_add(&fpsensor_dev->cdev, fpsensor_dev->devno, 1);
    if (error)
    {
        goto err2;
    }
    //register input device
#if FPSENSOR_INPUT
    fpsensor_dev->input = input_allocate_device();
    if (fpsensor_dev->input == NULL)
    {
        fpsensor_debug(ERR_LOG, "%s, Failed to allocate input device.\n", __func__);
        error = -ENOMEM;
        goto err2;
    }
    __set_bit(EV_KEY, fpsensor_dev->input->evbit);
    __set_bit(FPSENSOR_INPUT_HOME_KEY, fpsensor_dev->input->keybit);

    __set_bit(FPSENSOR_INPUT_MENU_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_INPUT_BACK_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_INPUT_FF_KEY, fpsensor_dev->input->keybit);

    __set_bit(FPSENSOR_NAV_TAP_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_NAV_UP_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_NAV_DOWN_KEY, fpsensor_dev->input->keyCHIPONE_DEV_NAMEbit);
    __set_bit(FPSENSOR_NAV_RIGHT_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_NAV_LEFT_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_INPUT_CAMERA_KEY, fpsensor_dev->input->keybit);
    fpsensor_dev->input->name = FPSENSOR_INPUT_NAME;
    if (input_register_device(fpsensor_dev->input))
    {
        fpsensor_debug(ERR_LOG, "%s, Failed to register input device.\n", __func__);
        error = -ENODEV;
        goto err1;
    }
#endif
    fpsensor_dev->device_available = 1;
    fpsensor_dev->irq_count = 1;
    // mt_eint_unmask(fpsensor_dev->irq);

    fpsensor_dev->sig_count = 0;

    fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);


    fpsensor_dev->probe_finish = 1;
    fpsensor_dev->is_sleep_mode = 0;
//fpsensor_hw_power_enable(1);
    fpsensor_spi_clk_enable(1);

    //init wait queue
    init_waitqueue_head(&fpsensor_dev->wq_irq_return);

    fpsensor_debug(INFO_LOG, "%s probe finished, normal driver version: %s\n", __func__,
                   FPSENSOR_SPI_VERSION);

    full_fp_chip_name(CHIPONE_DEV_INFO);
    FUNC_EXIT();
    return 0;
err1:
#if FPSENSOR_INPUT
    input_free_device(fpsensor_dev->input);
#endif

err2:
    device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
    //fpsensor_hw_power_enable(0);
    fpsensor_spi_clk_enable(0);
    kfree(fpsensor_dev);
    FUNC_EXIT();
    return status;
}

#if defined(USE_SPI_BUS)
static int fpsensor_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int fpsensor_remove(struct platform_device *spi)
#endif
{
#if defined(USE_PLATFORM_BUS)
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
#elif defined(USE_SPI_BUS)
	fpsensor_data_t *fpsensor_dev = spi_get_drvdata(spi);
#endif

    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if (fpsensor_dev->irq)
    {
        free_irq(fpsensor_dev->irq, fpsensor_dev);
    }

    fpsensor_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
    unregister_chrdev_region(fpsensor_dev->devno, 1);
    class_destroy(fpsensor_dev->class);
    if (fpsensor_dev->users == 0)
    {
#if FPSENSOR_INPUT
        if (fpsensor_dev->input != NULL)
        {
            input_unregister_device(fpsensor_dev->input);
        }
#endif
    }
    //fpsensor_hw_power_enable(0);

    fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
    kfree(fpsensor_dev);
#if 0
    if (s_DEVINFO_fpsensor != NULL)
    {
        kfree(s_DEVINFO_fpsensor);
    }
#endif     
    FUNC_EXIT();
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id fpsensor_of_match[] =
{
    { .compatible = "mediatek,fingerprint", },
    {}
};
MODULE_DEVICE_TABLE(of, fpsensor_of_match);
#endif
struct spi_device_id fpsensor_spi_id_table = {FPSENSOR_DEV_NAME, 0};

#if defined(USE_SPI_BUS)
static struct spi_driver fpsensor_spi_driver =
{
    .driver = {
        .name = FPSENSOR_DEV_NAME,
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = fpsensor_of_match,
#endif
    },
    .id_table = &fpsensor_spi_id_table,
    .probe = fpsensor_probe,
    .remove = fpsensor_remove,
};
#elif defined(USE_PLATFORM_BUS)
/*
static const struct dev_pmm fpsensor_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};
*/
static struct platform_driver fpsensor_plat_driver = {
	.probe = fpsensor_probe,
	.remove = fpsensor_remove,
	.shutdown = NULL,
	.driver = {
		   .name = FPSENSOR_DEV_NAME,
		   //.pm = &fpsensor_pm_ops,
		   .owner = THIS_MODULE,
		    //.bus	= &spi_bus_type,
#ifdef CONFIG_OF
           .of_match_table = fpsensor_of_match,
#endif /* CONFIG_OF */
	},
};
#endif

static int __init fpsensor_init(void)
{
    int status;

    FUNC_ENTRY();

#if defined(USE_PLATFORM_BUS)
	status = platform_driver_register(&fpsensor_plat_driver);
#elif defined(USE_SPI_BUS)
#if FPSENSOR_SPI_BUS_DYNAMIC
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
#endif
    status = spi_register_driver(&fpsensor_spi_driver);
#endif

    if (status < 0)
    {
        fpsensor_debug(ERR_LOG, "%s, Failed to register SPI driver.\n", __func__);
    }

    FUNC_EXIT();
    return status;
}
module_init(fpsensor_init);

static void __exit fpsensor_exit(void)
{
    FUNC_ENTRY();

#if defined(USE_PLATFORM_BUS)
	platform_driver_unregister(&fpsensor_plat_driver);
#elif defined(USE_SPI_BUS)
	spi_unregister_driver(&fpsensor_spi_driver);
#endif

    FUNC_EXIT();
}
module_exit(fpsensor_exit);

MODULE_AUTHOR("xhli");
MODULE_DESCRIPTION(" Fingerprint chip TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fpsensor_spi");
