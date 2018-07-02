/*bct3253 led controller driver*/

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
#include <asm/atomic.h>
#include <linux/leds.h>

#include <linux/sched.h>
#include <linux/kthread.h>

#define led_debug(fmt, args...) pr_debug(fmt, ##args)

typedef struct
{
    int led_port;
    int led_mode;
    int state;
    int max_current;
    int max_brightness;
    int mid_brightness;
    int rise_time;
    int fall_time;
    int rise_speed;
    int fall_speed;
} Led_Ctl_Data;

#define LED_WORK_DELAY  3

typedef char  kal_uint8 ;

static struct workqueue_struct *bct3253_wq_led;
struct delayed_work bct3253_work_led;

/*DEF REG ADDR*/
#define LED_MODE_STATE_REG            0x01
#define BASE_MAX_CURRENT_REG        0x03
#define BASE_RISE_FALL_REG              0x06
#define BASE_BRIGHTNESS_REG           0x09
#define BASE_RISE_SPEED_REG           0x0b
#define BASE_FALL_SPEED_REG           0x0c
#define LED_PORT1_REG_OFFSET         0
#define LED_PORT2_REG_OFFSET         1
#define LED_PORT3_REG_OFFSET         2

#define LED_CURREN_1MA  0x4
#define RF_TIME_1S   0x2
#define RF_SPEED_4MS  0x1
/*END*/

//static Led_Ctl_Data g_led_ctl_data;
static Led_Ctl_Data g_led_ctl_data_red;
static Led_Ctl_Data g_led_ctl_data_green;
static Led_Ctl_Data g_led_ctl_data_blue;
static int bct3253_int_flag=0;

struct led_classdev     red_led_cdev;
struct led_classdev     green_led_cdev;
struct led_classdev     blue_led_cdev;
struct led_classdev     red_breath_led_cdev;
struct led_classdev     green_breath_led_cdev;
struct led_classdev     blue_breath_led_cdev;
struct led_classdev     red_blink_led_cdev;
struct led_classdev     green_blink_led_cdev;
struct led_classdev     blue_blink_led_cdev;

#ifdef I9051_PR1_BOARD
enum
{
    BLUE_LED_PORT=1,
    GREEN_LED_PORT,
    RED_LED_PORT,
};
#else
enum
{
    BLUE_LED_PORT=1,
    RED_LED_PORT,
    GREEN_LED_PORT,
};
#endif //I9051_PR1_BOARD

enum
{
    LED_MODE_NOR=0,
    LED_MODE_BREATH,
    LED_MODE_BLINK=LED_MODE_BREATH,
};

//#define DEBUG_LEDS_PARA

static int register_leds(struct device *dev);

static DEFINE_MUTEX(bct3253_i2c_access);

#ifdef DEBUG_LEDS_PARA

static  int led_para[9][7]=
{
    {32,15,8,9,1,17,17},        //red,   32 is current, means brightness, can be set 1--255.   but i think 128 is large enough.
    {32,15,8,9,1,17,17},      //green
    {32,15,8,9,1,17,17},      //blue
    {32,15,8,4,2,34,34},        //red_breath
    {32,15,8,4,2,34,34},        //green_breath
    {32,15,8,4,2,34,34},        //blue_breath
    {32,15,8,4,2,0,0},         //red_blink
    {32,15,8,4,2,0,0},        //green_blink
    {32,15,8,4,2,0,0},       //blue_blink
};

static ssize_t show_bct3253_para(struct device *dev,struct device_attribute *attr, char *buf)
{
    led_debug("[show_bct3253_para] \n");
    return sprintf(buf, "red:%d %d %d %d %d %d %d, green:%d %d %d %d %d %d %d, blue:%d %d %d %d %d %d %d,red_breath:%d %d %d %d %d %d %d, green_breath:%d %d %d %d %d %d %d, blue_breath:%d %d %d %d %d %d %d,red_blink:%d %d %d %d %d %d %d, green_blink:%d %d %d %d %d %d %d, blue_blink:%d %d %d %d %d %d %d,\n",
                   led_para[0][0],led_para[0][1],led_para[0][2],led_para[0][3],led_para[0][4],led_para[0][5],led_para[0][6],   \
                   led_para[1][0],led_para[1][1],led_para[1][2],led_para[1][3],led_para[1][4],led_para[1][5],led_para[1][6],   \
                   led_para[2][0],led_para[2][1],led_para[2][2],led_para[2][3],led_para[2][4],led_para[2][5],led_para[2][6],   \
                   led_para[3][0],led_para[3][1],led_para[3][2],led_para[3][3],led_para[3][4],led_para[3][5],led_para[3][6],   \
                   led_para[4][0],led_para[4][1],led_para[4][2],led_para[4][3],led_para[4][4],led_para[4][5],led_para[4][6],   \
                   led_para[5][0],led_para[5][1],led_para[5][2],led_para[5][3],led_para[5][4],led_para[5][5],led_para[5][6],   \
                   led_para[6][0],led_para[6][1],led_para[6][2],led_para[6][3],led_para[6][4],led_para[6][5],led_para[6][6],   \
                   led_para[7][0],led_para[7][1],led_para[7][2],led_para[7][3],led_para[7][4],led_para[7][5],led_para[7][6],   \
                   led_para[8][0],led_para[8][1],led_para[8][2],led_para[8][3],led_para[8][4],led_para[8][5],led_para[8][6]);
}

static ssize_t store_bct3253_para(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    led_debug("[store_bct3253_para] \n");
    sscanf(buf ,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
           &(led_para[0][0]),&(led_para[0][1]),&(led_para[0][2]),&(led_para[0][3]),&(led_para[0][4]),&(led_para[0][5]),&(led_para[0][6]), \
           &(led_para[1][0]),&(led_para[1][1]),&(led_para[1][2]),&(led_para[1][3]),&(led_para[1][4]),&(led_para[1][5]),&(led_para[1][6]), \
           &(led_para[2][0]),&(led_para[2][1]),&(led_para[2][2]),&(led_para[2][3]),&(led_para[2][4]),&(led_para[2][5]),&(led_para[2][6]), \
           &(led_para[3][0]),&(led_para[3][1]),&(led_para[3][2]),&(led_para[3][3]),&(led_para[3][4]),&(led_para[3][5]),&(led_para[3][6]), \
           &(led_para[4][0]),&(led_para[4][1]),&(led_para[4][2]),&(led_para[4][3]),&(led_para[4][4]),&(led_para[4][5]),&(led_para[4][6]), \
           &(led_para[5][0]),&(led_para[5][1]),&(led_para[5][2]),&(led_para[5][3]),&(led_para[5][4]),&(led_para[5][5]),&(led_para[5][6]), \
           &(led_para[6][0]),&(led_para[6][1]),&(led_para[6][2]),&(led_para[6][3]),&(led_para[6][4]),&(led_para[6][5]),&(led_para[6][6]), \
           &(led_para[7][0]),&(led_para[7][1]),&(led_para[7][2]),&(led_para[7][3]),&(led_para[7][4]),&(led_para[7][5]),&(led_para[7][6]), \
           &(led_para[8][0]),&(led_para[8][1]),&(led_para[8][2]),&(led_para[8][3]),&(led_para[8][4]),&(led_para[8][5]),&(led_para[8][6]));

    return size;
}

static DEVICE_ATTR(bct3253_access, 0666, show_bct3253_para, store_bct3253_para); //664
#endif


static int bct3253_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
static struct i2c_client *new_client = NULL;
static const struct i2c_device_id bct3253_i2c_id[] = {{"bct3253",0},{}};

#ifdef GTP_CONFIG_OF
static const struct of_device_id bct_match_table[] =
{
    {.compatible ="bct,bct3253",},
    { },
};
#endif

static struct i2c_driver bct3253_driver =
{
    .driver = {
        .name    = "bct3253",
    },
    .probe       = bct3253_driver_probe,
    .id_table    = bct3253_i2c_id,
#ifdef GTP_CONFIG_OF
    .of_match_table = bct_match_table,
#endif
};

int bct3253_write_byte(kal_uint8 reg_addr, kal_uint8 reg_value)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;
    char    write_data[2] = {0};
    led_debug("bct3253_write_byte!reg_addr =0x%x reg_value=0x%x \n",reg_addr,reg_value);
    if(new_client==NULL)
    {
        led_debug("bct3253_write_byte!new_client==NULL !\n");
        return ret;
    }
    mutex_lock(&bct3253_i2c_access);
    write_data[0]=reg_addr;
    write_data[1]=reg_value;
    msg.flags = !I2C_M_RD;
    msg.addr  = new_client->addr;
    msg.len   = 2;
    msg.buf   = write_data;

    while(retries < 5)
    {
        ret = i2c_transfer(new_client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
        pr_err("bct3253_write_byte error!\n ");
    }
    mutex_unlock(&bct3253_i2c_access);

    return ret;
}
//EXPORT_SYMBOL(bct3253_write_byte);
int bct3253_read_byte(kal_uint8 reg_addr, kal_uint8 *ret_buf)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;
    mutex_lock(&bct3253_i2c_access);
    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = new_client->addr;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg_addr;

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = new_client->addr;
    msgs[1].len   = 1;
    msgs[1].buf   =ret_buf;

    while(retries < 5)
    {
        ret = i2c_transfer(new_client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
        pr_err("bct3253_read_byte!error!!\n");
    }
    mutex_unlock(&bct3253_i2c_access);
    return ret;
}

int bct3253_hw_init(void)
{
    int ret=0;
    ret=bct3253_write_byte(0,1);/*chip reset*/
    return ret;
}

void bct3253_hw_uninit(void)
{
}
//#define bct3253_BUSNUM 1
//static struct i2c_board_info __initdata i2c_bct3253 = { I2C_BOARD_INFO("bct3253", (0x30))};

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
#define LED_MODE_STATE_REG  0x01
#define BASE_MAX_CURRENT_REG        0x03
#define BASE_RISE_FALL_REG              0x06
#define BASE_BRIGHTNESS_REG           0x09
#define BASE_RISE_SPEED_REG           0x0b
#define BASE_FALL_SPEED_REG           0x0c
/*END*/

void  Update_Led_reg(void)
{
    char data=0x0;
    char mode;
    char state;
    int ret=-1;

    mode=((g_led_ctl_data_red.led_mode<<(RED_LED_PORT-1))|(g_led_ctl_data_blue.led_mode<<(BLUE_LED_PORT-1))|g_led_ctl_data_green.led_mode<<(GREEN_LED_PORT-1));                    //modify
    mode&=0x0f;
    state=((g_led_ctl_data_red.state<<(RED_LED_PORT-1))|(g_led_ctl_data_blue.state<<(BLUE_LED_PORT-1))|g_led_ctl_data_green.state<<(GREEN_LED_PORT-1));
    state&=0x0f;
    data=(mode<<4)|state;
    ret=bct3253_write_byte(LED_MODE_STATE_REG,data);
}

int set_led_reg(Led_Ctl_Data ctl_data)
{
    char data=0x0;
    int reg_offset=0;
    int ret=-1;
    int step=0;
    if(ctl_data.led_port>3)
    {
        ctl_data.led_port=3;
    }
    if(ctl_data.led_port<1)
    {
        ctl_data.led_port=1;
    }

    reg_offset=ctl_data.led_port-1;

    data=(ctl_data.fall_speed&0x00ff);
    ret= bct3253_write_byte(BASE_FALL_SPEED_REG+reg_offset*4,data);
    step++;
    if(ret<0)
    {
        pr_err("Set_Controller Failed at step %d \n",step);
        goto exit;
    }

    data=ctl_data.rise_speed&0x00ff;
    ret= bct3253_write_byte(BASE_RISE_SPEED_REG+reg_offset*4,data);
    step++;
    if(ret<0)
    {
        pr_err("Set_Controller Failed at step %d \n",step);
        goto exit;
    }

    data=((ctl_data.rise_time&0x0f)<<4 |(ctl_data.fall_time&0x0f));
    ret=bct3253_write_byte(BASE_RISE_FALL_REG+reg_offset,data);
    step++;
    if(ret<0)
    {
        pr_err("Set_Controller Failed at step %d \n",step);
        goto exit;
    }

    data=ctl_data.max_current;
    ret=bct3253_write_byte(2,0x40);//max current 25.5ma
    ret=bct3253_write_byte(BASE_MAX_CURRENT_REG+reg_offset,data);
    step++;
    if(ret<0)
    {
        pr_err("Set_Controller Failed at step %d \n",step);
        goto exit;
    }

    data=((ctl_data.max_brightness&0x0f)<<4 |(ctl_data.mid_brightness&0x0f));
    ret=bct3253_write_byte(BASE_BRIGHTNESS_REG+reg_offset*4,data);
    step++;
    if(ret<0)
    {
        pr_err("Set_Controller Failed at step %d \n",step);
        goto exit;
    }

    step++;
    if(ret<0)
    {
        pr_err("Set_Controller Failed at step %d \n",step);
        goto exit;
    }

exit:

    return ret;

}

static void bct3253_led_work_func(struct work_struct *work)
{
    led_debug("bct3253_led_work_func ! \n");
    if(bct3253_int_flag)
    {
        bct3253_write_byte(0,1);
        set_led_reg(g_led_ctl_data_green);
        set_led_reg(g_led_ctl_data_blue);
        set_led_reg(g_led_ctl_data_red);
        Update_Led_reg();
    }
    else
    {
        led_debug("bct3253_led_work_func bct3253_int_flag ==0 do nothing!\n");
    }
}

static int bct3253_controller_probe(struct platform_device *dev)
{
//   int ret_device_file = 0;
    led_debug("******** bct3253_controller_probe!! ********\n" );
    register_leds(&(dev->dev));
#ifdef DEBUG_LEDS_PARA
    device_create_file(&(dev->dev), &dev_attr_bct3253_access);
#endif
    return 0;
}

struct platform_device bct3253_controller_device =
{
    .name   = "bct3253_controller",
    .id     = -1,
};

static struct platform_driver bct3253_controller_driver =
{
    .probe      = bct3253_controller_probe,
    .driver     = {
        .name = "bct3253_controller",
    },
};

static int bct3253_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err=0;
    led_debug("[bct3253_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;

    if(bct3253_hw_init()<0)
    {
        bct3253_hw_uninit();
        kfree(new_client);
        new_client=NULL;
        err = -EINVAL;
        bct3253_int_flag=0;
        pr_err("[bct3253_driver_probe]  bct3253_hw_init   failed ! \n");
        goto exit;
    }

    INIT_DELAYED_WORK(&bct3253_work_led, bct3253_led_work_func);
    bct3253_wq_led = create_singlethread_workqueue("bct3253_wq_led");
    if(!bct3253_wq_led)
    {
        pr_err("bct3253_driver_probe create wq failed! \n");
        bct3253_hw_uninit();
        kfree(new_client);
        new_client=NULL;
        err = -EINVAL;
    }

    bct3253_int_flag=1;
exit:
    return err;
}

static void kernel_set_breath_led( int port , int mode , int state , int mcurrent , int max_b , int mid_b ,int rise_time ,int fall_time , int rspeed , int fspeed)
{
    int led_current = mcurrent;//calulate led current
    if(0 == bct3253_int_flag)
    {
        pr_err("kernel_set_breath_led  error: bct3253_int_flag!\n");
        return;
    }

    if(state>0)
    {
        state=1;
    }
    else
    {
        state=0;
    }

    if (led_current > 200)
        led_current = 200;//led support max current is 20ma
    led_debug("kernel_set_breath_led: port=%d,mode=%d, state=%d led_current=%dma!\n",port,mode, state,led_current/10);
    g_led_ctl_data_red.led_port = RED_LED_PORT;
    g_led_ctl_data_green.led_port = GREEN_LED_PORT; //bug for first use, because when first call green led,  but the  g_led_ctl_data_red.led_port=0=g_led_ctl_data_blue.led_port=g_led_ctl_data_green.led_port
    g_led_ctl_data_blue.led_port = BLUE_LED_PORT;

    if(port==RED_LED_PORT)
    {
        g_led_ctl_data_red.led_port=port;
        g_led_ctl_data_red.led_mode=mode;
        g_led_ctl_data_red.state=state;
        g_led_ctl_data_red.max_current=led_current;
        g_led_ctl_data_red.max_brightness=max_b;
        g_led_ctl_data_red.mid_brightness=mid_b;
        g_led_ctl_data_red.rise_time=rise_time;
        g_led_ctl_data_red.fall_time=fall_time;
        g_led_ctl_data_red.rise_speed=rspeed;
        g_led_ctl_data_red.fall_speed=fspeed;

    }
    else if(port==GREEN_LED_PORT)
    {
        g_led_ctl_data_green.led_port=port;
        g_led_ctl_data_green.led_mode=mode;
        g_led_ctl_data_green.state=state;
        g_led_ctl_data_green.max_current=led_current;
        g_led_ctl_data_green.max_brightness=max_b;
        g_led_ctl_data_green.mid_brightness=mid_b;
        g_led_ctl_data_green.rise_time=rise_time;
        g_led_ctl_data_green.fall_time=fall_time;
        g_led_ctl_data_green.rise_speed=rspeed;
        g_led_ctl_data_green.fall_speed=fspeed;

    }
    else if(port==BLUE_LED_PORT)
    {
        g_led_ctl_data_blue.led_port=port;
        g_led_ctl_data_blue.led_mode=mode;
        g_led_ctl_data_blue.state=state;
        g_led_ctl_data_blue.max_current=led_current;
        g_led_ctl_data_blue.max_brightness=max_b;
        g_led_ctl_data_blue.mid_brightness=mid_b;
        g_led_ctl_data_blue.rise_time=rise_time;
        g_led_ctl_data_blue.fall_time=fall_time;
        g_led_ctl_data_blue.rise_speed=rspeed;
        g_led_ctl_data_blue.fall_speed=fspeed;
    }
    queue_delayed_work(bct3253_wq_led, &bct3253_work_led,LED_WORK_DELAY);
}

static void red_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_NOR,value,led_para[0][0],led_para[0][1],led_para[0][2],led_para[0][3],led_para[0][4],led_para[0][5],led_para[0][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_NOR,value,value,15,8,9,1,17,17);
    //kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_NOR,0,88,15,8,9,1,17,17);
    //kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_NOR,0,88,15,8,9,1,17,17);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_NOR,value,88,15,8,9,1,17,17);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_NOR,value,1,15,8,9,1,17,17);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_NOR,value,35,15,8,9,1,17,17);
#endif /*CONFIG_PROJECT_I9051*/
#endif
}

static enum led_brightness red_led_brightness_get(struct led_classdev *cdev)
{
    if(g_led_ctl_data_red.led_mode == LED_MODE_NOR)
    {
        if(g_led_ctl_data_red.state !=0)
        {
            return g_led_ctl_data_red.max_current;
        }
    }
    return LED_OFF;
}

static void green_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_NOR,value,led_para[1][0],led_para[1][1],led_para[1][2],led_para[1][3],led_para[1][4],led_para[1][5],led_para[1][6]);
#else

#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_NOR,value,value,15,8,9,1,17,17);
    //kernel_set_breath_led(RED_LED_PORT,LED_MODE_NOR,0,88,15,8,9,1,17,17);
    //kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_NOR,0,88,15,8,9,1,17,17);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_NOR,value,125,15,8,9,1,17,17);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_NOR,value,37,15,8,9,1,17,17);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_NOR,value,8,15,8,9,1,17,17);
#endif
#endif
}

static enum led_brightness green_led_brightness_get(struct led_classdev *cdev)
{
    if(g_led_ctl_data_green.led_mode == LED_MODE_NOR)
    {
        if(g_led_ctl_data_green.state !=0)
        {
            return g_led_ctl_data_green.max_current;
        }
    }
    return LED_OFF;
}

static void blue_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_NOR,value,led_para[2][0],led_para[2][1],led_para[2][2],led_para[2][3],led_para[2][4],led_para[2][5],led_para[2][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_NOR,value,value,15,8,9,1,17,17);
    //kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_NOR,0,88,15,8,9,1,17,17);
    //kernel_set_breath_led(RED_LED_PORT,LED_MODE_NOR,0,88,15,8,9,1,17,17);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_NOR,value,51,15,8,9,1,17,17);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_NOR,value,48,15,8,9,1,17,17);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_NOR,value,59,15,8,9,1,17,17);
#endif /*CONFIG_PROJECT_I9051*/
#endif

}

static enum led_brightness blue_led_brightness_get(struct led_classdev *cdev)
{
    if(g_led_ctl_data_blue.led_mode == LED_MODE_NOR)
    {
        if(g_led_ctl_data_blue.state !=0)
        {
            return g_led_ctl_data_blue.max_current;
        }
    }
    return LED_OFF;
}

static void red_breath_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,led_para[3][0],led_para[3][1],led_para[3][2],led_para[3][3],led_para[3][4],led_para[3][5],led_para[3][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,value,15,8,7,4,68,68);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,88,15,8,7,4,68,68);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,1,15,8,7,4,68,68);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,35,15,8,7,4,68,68);
#endif
#endif


}

static enum led_brightness red_breath_led_brightness_get(struct led_classdev *cdev)
{
    if(g_led_ctl_data_red.led_mode == LED_MODE_BREATH)
    {
        if(g_led_ctl_data_red.state !=0)
        {
            return g_led_ctl_data_red.max_current;
        }
    }
    return LED_OFF;
}

static void green_breath_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,led_para[4][0],led_para[4][1],led_para[4][2],led_para[4][3],led_para[4][4],led_para[4][5],led_para[4][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,value,15,8,7,4,68,68);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,125,15,8,7,4,68,68);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,37,15,8,7,4,68,68);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,8,15,8,7,4,68,68);
#endif

#endif
}

static enum led_brightness green_breath_led_brightness_get(struct led_classdev *cdev)
{
    if(g_led_ctl_data_green.led_mode == LED_MODE_BREATH)
    {
        if(g_led_ctl_data_green.state !=0)
        {
            return g_led_ctl_data_green.max_current;
        }
    }
    return LED_OFF;
}

static void blue_breath_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,led_para[5][0],led_para[5][1],led_para[5][2],led_para[5][3],led_para[5][4],led_para[5][5],led_para[5][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,value,15,8,7,4,68,68);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,51,15,8,7,4,68,68);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,48,15,7,8,4,68,68);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,59,15,8,7,4,68,68);
#endif
#endif
}

static enum led_brightness blue_breath_led_brightness_get(struct led_classdev *cdev)
{
    if(g_led_ctl_data_blue.led_mode == LED_MODE_BREATH)
    {
        if(g_led_ctl_data_blue.state !=0)
        {
            return g_led_ctl_data_blue.max_current;
        }
    }
    return LED_OFF;
}

static void red_blink_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,led_para[6][0],led_para[6][1],led_para[6][2],led_para[6][3],led_para[6][4],led_para[6][5],led_para[6][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,value,15,8,6,1,0,0);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,88,15,8,6,1,0,0);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,1,15,8,6,1,0,0);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,35,15,8,6,1,0,0);
#endif
#endif

}

static void green_blink_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,led_para[7][0],led_para[7][1],led_para[7][2],led_para[7][3],led_para[7][4],led_para[7][5],led_para[7][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,value,15,8,6,1,0,0);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,125,15,8,6,1,0,0);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,37,15,8,6,1,0,0);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,8,15,8,6,1,0,0);
#endif
#endif
}

static void blue_blink_led_brightness_set(struct led_classdev *cdev, enum led_brightness value)
{
#ifdef DEBUG_LEDS_PARA
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,led_para[8][0],led_para[8][1],led_para[8][2],led_para[8][3],led_para[8][4],led_para[8][5],led_para[8][6]);
#else
#ifdef CONFIG_PROJECT_I9051
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,value,15,8,6,1,0,0);
#else
    kernel_set_breath_led(RED_LED_PORT,LED_MODE_BREATH,value,51,15,8,6,1,0,0);
    kernel_set_breath_led(GREEN_LED_PORT,LED_MODE_BREATH,value,48,15,8,6,1,0,0);
    kernel_set_breath_led(BLUE_LED_PORT,LED_MODE_BREATH,value,59,15,8,6,1,0,0);
#endif
#endif
}


static int register_leds(struct device *dev)
{
    int rc;

    red_led_cdev.name = "red";
    red_led_cdev.brightness_set =red_led_brightness_set;
    red_led_cdev.brightness_get =red_led_brightness_get;
    rc = led_classdev_register(dev, &red_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register red led, rc=%d\n",rc);
        goto failed_unregister_led_R;
    }

    red_blink_led_cdev.name = "red_blink";
    red_blink_led_cdev.brightness_set =red_blink_led_brightness_set;
    red_blink_led_cdev.brightness_get =red_breath_led_brightness_get;
    rc = led_classdev_register(dev, &red_blink_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register red_blink led, rc=%d\n",rc);
        goto failed_unregister_led_R_BLINK;
    }

    red_breath_led_cdev.name = "red_breath";
    red_breath_led_cdev.brightness_set =red_breath_led_brightness_set;
    red_breath_led_cdev.brightness_get =red_breath_led_brightness_get;
    rc = led_classdev_register(dev, &red_breath_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register red_breath led, rc=%d\n",rc);
        goto failed_unregister_led_R_BREATH;
    }

    green_led_cdev.name = "green";
    green_led_cdev.brightness_set =green_led_brightness_set;
    green_led_cdev.brightness_get =green_led_brightness_get;
    rc = led_classdev_register(dev, &green_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register green led, rc=%d\n",rc);
        goto failed_unregister_led_G;
    }

    green_blink_led_cdev.name = "green_blink";
    green_blink_led_cdev.brightness_set =green_blink_led_brightness_set;
    green_blink_led_cdev.brightness_get =green_breath_led_brightness_get;
    rc = led_classdev_register(dev, &green_blink_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register green_blink led, rc=%d\n",rc);
        goto failed_unregister_led_G_BLINK;
    }

    green_breath_led_cdev.name = "green_breath";
    green_breath_led_cdev.brightness_set =green_breath_led_brightness_set;
    green_breath_led_cdev.brightness_get =green_breath_led_brightness_get;
    rc = led_classdev_register(dev, &green_breath_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register green_breath led, rc=%d\n",rc);
        goto failed_unregister_led_G_BREATH;
    }

    blue_led_cdev.name = "blue";
    blue_led_cdev.brightness_set =blue_led_brightness_set;
    blue_led_cdev.brightness_get =blue_led_brightness_get;
    rc = led_classdev_register(dev, &blue_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register blue led, rc=%d\n",rc);
        goto failed_unregister_led_B;
    }

    blue_blink_led_cdev.name = "blue_blink";
    blue_blink_led_cdev.brightness_set =blue_blink_led_brightness_set;
    blue_blink_led_cdev.brightness_get =blue_breath_led_brightness_get;
    rc = led_classdev_register(dev, &blue_blink_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register blue_blink led, rc=%d\n",rc);
        goto failed_unregister_led_B_BLINK;
    }

    blue_breath_led_cdev.name = "blue_breath";
    blue_breath_led_cdev.brightness_set =blue_breath_led_brightness_set;
    blue_breath_led_cdev.brightness_get =blue_breath_led_brightness_get;
    rc = led_classdev_register(dev, &blue_breath_led_cdev);
    if (rc<0)
    {
        dev_err(dev, "unable to register blue_breath led, rc=%d\n",rc);
        goto failed_unregister_led_B_BREATH;
    }
    return 0;

failed_unregister_led_B_BREATH:
    led_classdev_unregister(&blue_blink_led_cdev);
failed_unregister_led_B_BLINK:
    led_classdev_unregister(&blue_led_cdev);
failed_unregister_led_B:
    led_classdev_unregister(&green_breath_led_cdev);
failed_unregister_led_G_BREATH:
    led_classdev_unregister(&green_blink_led_cdev);
failed_unregister_led_G_BLINK:
    led_classdev_unregister(&green_led_cdev);
failed_unregister_led_G:
    led_classdev_unregister(&red_breath_led_cdev);
failed_unregister_led_R_BREATH:
    led_classdev_unregister(&red_blink_led_cdev);
failed_unregister_led_R_BLINK:
    led_classdev_unregister(&red_led_cdev);
failed_unregister_led_R:

    return rc;
}

static void unregister_leds(void)
{
    led_classdev_unregister(&blue_breath_led_cdev);
    led_classdev_unregister(&blue_blink_led_cdev);
    led_classdev_unregister(&blue_led_cdev);
    led_classdev_unregister(&green_breath_led_cdev);
    led_classdev_unregister(&green_blink_led_cdev);
    led_classdev_unregister(&green_led_cdev);
    led_classdev_unregister(&red_breath_led_cdev);
    led_classdev_unregister(&red_blink_led_cdev);
    led_classdev_unregister(&red_led_cdev);
}

static int __init bct3253_init(void)
{
    int ret=0;
    led_debug("[bct3253_init] init start\n");

    ret = i2c_add_driver(&bct3253_driver);
    if(ret !=0)
    {
        pr_err("[bct3253_init] failed to register bct3253 i2c driver.\n");
        goto out;
    }

    ret = platform_driver_register(&bct3253_controller_driver);
    if (ret < 0)
    {
        pr_err("****[bct3253_init] Unable to register driver (%d)\n", ret);
        i2c_del_driver(&bct3253_driver);
        goto out;
    }

    ret = platform_device_register(&bct3253_controller_device);
    if (ret < 0)
    {
        pr_err("****[bct3253_init] Unable to device register(%d)\n", ret);
        platform_driver_unregister(&bct3253_controller_driver);
        i2c_del_driver(&bct3253_driver);
    }

    led_debug("[bct3253_init] init end ret %d\n",ret);
out:
    return ret;
}

static void __exit bct3253_exit(void)
{
    unregister_leds();
    i2c_del_driver(&bct3253_driver);
    cancel_delayed_work(&bct3253_work_led);
    platform_driver_unregister(&bct3253_controller_driver);
    platform_device_unregister(&bct3253_controller_device);
}

module_init(bct3253_init);
module_exit(bct3253_exit);
//MODULE_DESCRIPTION("BCT3253 Driver");
//MODULE_LICENSE("GPL");
