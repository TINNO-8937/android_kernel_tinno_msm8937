#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>

//#define CONFIG_DEV_INFO_DEBUG

#ifdef CDBG
#undef CDBG
#endif

#ifdef CONFIG_DEV_INFO_DEBUG
#define CDBG(fmt, args...) pr_err("[dev_info] " fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define INFO_LEN  80

static struct kobject *dev_info;

static char main_camera[INFO_LEN];
static char sub_camera[INFO_LEN];
#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
static char aux_camera[INFO_LEN];
#endif
static char lcd_info[INFO_LEN];
static char tp_info[INFO_LEN];
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if defined(CONFIG_PROJECT_L5421)
int sunpanle_enbale = 0;
#endif
//ENE <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun

#ifdef CONFIG_DEV_INFO
#define DEV_INFO_LEN 80
static char main_camera_and_eeprom[DEV_INFO_LEN];
static char sub_camera_and_eeprom[DEV_INFO_LEN];
#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
static char aux_camera_and_eeprom[DEV_INFO_LEN];
#endif
#endif

enum {
	MAIN_CAMERA, 
 	SUB_CAMERA, 
#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
    AUX_CAMERA,
#endif
};

static int set_dev_info(const char *const str, int which_dev)
{
	int ret;
	ret = 0;

	if(str ==  NULL){
		CDBG("%s : str is NULL\n", __func__);
		return -1;
	}
	CDBG("%s : which_dev=%d\n", __func__, which_dev);
	switch(which_dev){
		case MAIN_CAMERA:
			ret = snprintf(main_camera, INFO_LEN, "%s", str);
			break;
		case SUB_CAMERA:
			ret = snprintf(sub_camera, INFO_LEN, "%s", str);
			break;
#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
		case AUX_CAMERA:
			ret = snprintf(aux_camera, INFO_LEN, "%s", str);
			break;
#endif
		default:
			break;
	}

	return ret;
}

int store_main_camera_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return set_dev_info(str, MAIN_CAMERA);
}
EXPORT_SYMBOL_GPL(store_main_camera_info);

int store_sub_camera_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return set_dev_info(str, SUB_CAMERA);
}
EXPORT_SYMBOL_GPL(store_sub_camera_info);

#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
int store_aux_camera_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return set_dev_info(str, AUX_CAMERA);
}
EXPORT_SYMBOL_GPL(store_aux_camera_info);
#endif


static ssize_t main_camera_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	CDBG("%s : main_camera=%s\n", __func__, main_camera);
	return snprintf(buf, INFO_LEN, "%s\n", main_camera);
}

static ssize_t sub_camera_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : sub_camera=%s\n", __func__, sub_camera);
	return snprintf(buf, INFO_LEN, "%s\n", sub_camera);
}

#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
static ssize_t aux_camera_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : aux_camera=%s\n", __func__, aux_camera);
	return snprintf(buf, INFO_LEN, "%s\n", aux_camera);
}
#endif

int store_lcd_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(lcd_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_lcd_info);
static ssize_t lcd_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : lcd_info=%s\n", __func__, lcd_info);
	return snprintf(buf, INFO_LEN, "%s\n", lcd_info);
}

int store_tp_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(tp_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_tp_info);
static ssize_t tp_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : tp_info=%s\n", __func__, tp_info);
	return snprintf(buf, INFO_LEN, "%s\n", tp_info);
}
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if defined(CONFIG_SUN_PANEL_FUNCTION)
static ssize_t sun_panle_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : sun_panle=%d\n", __func__, sunpanle_enbale);
	return snprintf(buf, INFO_LEN, "%d\n", sunpanle_enbale);
}
static ssize_t sun_panle_store(struct kobject *kobj, struct kobj_attribute *attr,
		      const char *buf, size_t count)
{
       CDBG("%s : sun_panle=%s\n", __func__, buf);
       sunpanle_enbale = buf[0] - 0x30;
	return count;//snprintf(sun_panle, INFO_LEN, "%s\n",buf );
}


int show_sub_sun_panle(void)
{
       CDBG("%s : sun_panle=%d\n", __func__, sunpanle_enbale);
	return sunpanle_enbale;//(sun_panle[0]!=0x30)?1:0;
}
EXPORT_SYMBOL_GPL(show_sub_sun_panle);
#endif
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun

#if 0
int store_sub_camera_eeprom_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(sub_camera_eeprom_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_sub_camera_eeprom_info);
static ssize_t sub_camera_eeprom_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : sub_camera_eeprom_info=%s\n", __func__, sub_camera_eeprom_info);
	return snprintf(buf, INFO_LEN, "%s\n", sub_camera_eeprom_info);
}

int store_main_camera_eeprom_info(const char *const str)
{
	CDBG("%s : str=%s\n", __func__, str);
	return snprintf(main_camera_eeprom_info, INFO_LEN, "%s", str);
}
EXPORT_SYMBOL_GPL(store_main_camera_eeprom_info);
static ssize_t main_camera_eeprom_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	CDBG("%s : tp_info=%s\n", __func__, main_camera_eeprom_info);
	return snprintf(buf, INFO_LEN, "%s\n", main_camera_eeprom_info);
}
#endif

static struct kobj_attribute main_camera_attribute =
	__ATTR(main_camera, 0555, main_camera_show, NULL);

static struct kobj_attribute sub_camera_attribute =
	__ATTR(sub_camera, 0555, sub_camera_show, NULL);

#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
static struct kobj_attribute aux_camera_attribute =
	__ATTR(aux_camera, 0555, aux_camera_show, NULL);

#endif
static struct kobj_attribute lcd_info_attribute =
	__ATTR(lcd_info, 0555, lcd_info_show, NULL);

static struct kobj_attribute tp_info_attribute =
	__ATTR(tp_info, 0555, tp_info_show, NULL);

//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if defined(CONFIG_PROJECT_L5421)
static struct kobj_attribute sun_panle_attribute =
	__ATTR(sun_panle, 0666, sun_panle_show, sun_panle_store);
#endif
//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if 0
static struct kobj_attribute main_camera_eeprom_info_attribute =
	__ATTR(main_camera_eeprom_info, 0555, main_camera_eeprom_info_show, NULL);

static struct kobj_attribute sub_camera_eeprom_info_attribute =
	__ATTR(sub_camera_eeprom_info, 0555, sub_camera_eeprom_info_show, NULL);
#endif

static struct attribute *attrs[] = {
	&main_camera_attribute.attr,
	&sub_camera_attribute.attr,
#if defined(CONFIG_CAMERA_AUX_INFO)//add by lhm 20170601
	&aux_camera_attribute.attr,
#endif
	&lcd_info_attribute.attr,
	&tp_info_attribute.attr,
	//Begin <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
	#if defined(CONFIG_PROJECT_L5421)
	&sun_panle_attribute.attr,
	#endif
	//END <REQ><JABALL-1500><20150623>Add sun panel function for L5510;xiongdajun
#if 0	
	&main_camera_eeprom_info_attribute.attr,
	&sub_camera_eeprom_info_attribute.attr,	
#endif	
	NULL,
};


static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init dev_info_init(void)
{
	int retval;

	dev_info = kobject_create_and_add("dev_info", kernel_kobj);
	if (!dev_info){
		pr_err("%s : kobject_create_and_add failed.\n", __func__);
		return -ENOMEM;
	}

	retval = sysfs_create_group(dev_info, &attr_group);
	if (retval){
		pr_err("%s : sysfs_create_group failed.\n", __func__);
		kobject_put(dev_info);
	}
	return retval;
}

void store_camera_info(const char *const sensor_name, const char *const eeprom_name)
{
    #ifdef CONFIG_DEV_INFO
        int32_t rc = 0;
	if(!strncmp(sensor_name, "s5k3p3", sizeof("s5k3p3")))
	{
		rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(16M)", 
			"s5k3p3");		
		if(rc < 0){
			store_main_camera_info(sensor_name);
		}else{
			store_main_camera_info(sub_camera_and_eeprom);
		}
	}
       else if(!strncmp(sensor_name, "s5k4h8", sizeof("s5k4h8")))
	{
	        if(!strncmp(eeprom_name, "sunny_s5k4h8", sizeof("sunny_s5k4h8"))){
			//store_sub_camera_eeprom_info("DaLing");

			rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)", 
					"s5k4h8_sunny");		
		}
		if(rc < 0){
			store_sub_camera_info(sensor_name);
		}else{
			store_sub_camera_info(main_camera_and_eeprom);
		}
	}
        else if(!strncmp(sensor_name, "s5k4h80x11", sizeof("s5k4h80x11")))
	{
	        if(!strncmp(eeprom_name, "sunny_s5k4h80x11", sizeof("sunny_s5k4h80x11"))){
			//store_sub_camera_eeprom_info("DaLing");

			rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)", 
					"s5k4h80x11_sunny");		
		}
		if(rc < 0){
			store_sub_camera_info(sensor_name);
		}else{
			store_sub_camera_info(main_camera_and_eeprom);
		}
	}
//Ramiel add p7203 camera +++++++++++++++
	  else if(!strncmp(sensor_name, "p7203_gb_imx258", sizeof("p7203_gb_imx258")))
	 {
		  rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)", 
			  "imx258_guangbao");	  
		  if(rc < 0){
			  store_main_camera_info(sensor_name);
		  }else{
			  store_main_camera_info(main_camera_and_eeprom);
		  }
	 
	 }
	  else if(!strncmp(sensor_name, "p7203_sy_imx258", sizeof("p7203_sy_imx258")))
	  {
		  rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)", 
			  "imx258_sunny");	  
		  if(rc < 0){
			  store_main_camera_info(sensor_name);
		  }else{
			  store_main_camera_info(main_camera_and_eeprom);
		  }
	  
	  }
	  else if(!strncmp(sensor_name, "p7203_gb_s5k4h8", sizeof("p7203_gb_s5k4h8")))
	 {
		 rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)", 
			 "s5k4h8_guangbao"); 	 
		 if(rc < 0){
			 store_sub_camera_info(sensor_name);
		 }else{
			 store_sub_camera_info(sub_camera_and_eeprom);
		 }

	 }
//Ramiel add p7203 camera  ---------------------

	 else if(!strncmp(sensor_name, "p7705_sunny_s5k4h8", sizeof("p7705_sunny_s5k4h8")))
	{
		 rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)", 
			 "s5k4h8_sunny"); 	 
		 if(rc < 0){
			 store_sub_camera_info(sensor_name);
		 }else{
			 store_sub_camera_info(sub_camera_and_eeprom);
		 }

	}
	 else if(!strncmp(sensor_name, "p7705_sunny_0x11_s5k4h8", sizeof("p7705_sunny_0x11_s5k4h8")))
	{
		 rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)", 
			 "s5k4h8_Blue_sunny"); 	 
		 if(rc < 0){
			 store_sub_camera_info(sensor_name);
		 }else{
			 store_sub_camera_info(sub_camera_and_eeprom);
		 }

	}
	 else if(!strncmp(sensor_name, "imx258", sizeof("imx258")))
	{
	        if(!strncmp(eeprom_name, "p7705_sunny_imx258_otp", sizeof("p7705_sunny_imx258_otp"))){
			//store_sub_camera_eeprom_info("DaLing");

			rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)", 
					"imx258_sunny");		
		}
		if(rc < 0){
			store_main_camera_info(sensor_name);
		}else{
			store_main_camera_info(main_camera_and_eeprom);
		}
	}
       //BEGIN<20160617><add camera info for 7201>wangyanhui 
	else if(!strncmp(sensor_name, "imx258_guangbao_p7201", sizeof("imx258_guangbao_p7201")))
	{
		rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)", 
			"imx258_guangbao");		
		if(rc < 0){
			store_main_camera_info(sensor_name);
		}else{
			store_main_camera_info(main_camera_and_eeprom);
		}
	}	 
	else if(!strncmp(sensor_name, "imx258_sunny_p7201", sizeof("imx258_sunny_p7201")))
	{
		rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)", 
			"imx258_sunny");		
		if(rc < 0){
			store_main_camera_info(sensor_name);
		}else{
			store_main_camera_info(main_camera_and_eeprom);
		}
	}		
	 else if(!strncmp(sensor_name, "s5k4h8_p7201", sizeof("s5k4h8_p7201")))
	{
		 rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)", 
			 "s5k4h8"); 	 
		 if(rc < 0){
			 store_sub_camera_info(sensor_name);
		 }else{
			 store_sub_camera_info(sub_camera_and_eeprom);
		 }

	}
	 else if(!strncmp(sensor_name, "ov8856", sizeof("ov8856")))
	{
		 rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)", "ov8856");
		 if(rc < 0){
			 store_sub_camera_info(sensor_name);
		 }else{
			 store_sub_camera_info(sub_camera_and_eeprom);
		 }

	}
       //END<20160617><add camera info for 7201>wangyanhui	 
     //add camera info for v12bnlite by lhm
      else if(!strncmp(sensor_name, "ov13855_ofilm_v12bnlite", sizeof("ov13855_ofilm_v12bnlite")))
        {
                 rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)",
                         "ov13855");
                 if(rc < 0){
                         store_main_camera_info(sensor_name);
                 }else{
                         store_main_camera_info(main_camera_and_eeprom);
                 }

        }

      else if(!strncmp(sensor_name, "s5k3l8_sunny_v12bnlite", sizeof("s5k3l8_sunny_v12bnlite")))
	{
		 rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)", 
			 "s5k3l8"); 	 
		 if(rc < 0){
			 store_main_camera_info(sensor_name);
		 }else{
			 store_main_camera_info(main_camera_and_eeprom);
		 }

	}
	 else if(!strncmp(sensor_name, "s5k3p3st_sunny_v12bnlite", sizeof("s5k3p3st_sunny_v12bnlite")))
	{
		 rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(16M)", "s5k3p3st");
		 if(rc < 0){
			 store_sub_camera_info(sensor_name);
		 }else{
			 store_sub_camera_info(sub_camera_and_eeprom);
		 }

	}
	 //end
     //add camera info for c800 by wzz
      else if(!strncmp(sensor_name, "s5k3p3sm_sunny_c800", sizeof("s5k3p3sm_sunny_c800")))
        {
                 rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(16M)",
                         "s5k3p3");
                 if(rc < 0){
                         store_main_camera_info(sensor_name);
                 }else{
                         store_main_camera_info(main_camera_and_eeprom);
                 }

        }
      else if(!strncmp(sensor_name, "s5k3l8_sunny_c800", sizeof("s5k3l8_sunny_c800")))
        {
                 rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(13M)",
                         "s5k3l8");
                 if(rc < 0){
                         store_main_camera_info(sensor_name);
                 }else{
                         store_main_camera_info(main_camera_and_eeprom);
                 }

        }

	 else if(!strncmp(sensor_name, "s5k3p8sp_ofilm_c800", sizeof("s5k3p8sp_ofilm_c800")))
	{
		 rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(16M)", "s5k3p8");
		 if(rc < 0){
			 store_sub_camera_info(sensor_name);
		 }else{
			 store_sub_camera_info(sub_camera_and_eeprom);
		 }

	}
	 //end
	 //Begin <REQ><><20171130>add camera info for c860
        else if(!strncmp(sensor_name, "imx499_ofilm_c860", sizeof("imx499_ofilm_c860")))
        {
            rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_(16M)",
                "imx499_ofilm");
            if(rc < 0){
              store_main_camera_info(sensor_name);
            }else{
              store_main_camera_info(main_camera_and_eeprom);
            }
        }
        else if(!strncmp(sensor_name, "s5k3p8sp_sunny_c860", sizeof("s5k3p8sp_sunny_c860")))
        {
            rc = snprintf(sub_camera_and_eeprom, DEV_INFO_LEN, "%s_(16M)",
                "s5k3p8sp_sunny");
            if(rc < 0){
              store_sub_camera_info(sensor_name);
            }else{
              store_sub_camera_info(sub_camera_and_eeprom);
            }
        }
        #if defined(CONFIG_CAMERA_AUX_INFO)
	  else if(!strncmp(sensor_name, "ov8856_sunny_c860", sizeof("ov8856_sunny_c860")))
	 {
            rc = snprintf(aux_camera_and_eeprom, DEV_INFO_LEN, "%s_(8M)",
                "ov8856_sunny");
            if(rc < 0){
             store_aux_camera_info(sensor_name);
            }else{
             store_aux_camera_info(aux_camera_and_eeprom);
            }
	 }
      #endif
      //End <REQ><><20171130>add camera info for c860
	else
	{
		if(!strncmp(eeprom_name, "daling_p5v23c", sizeof("daling_p5v23c"))){
			rc = snprintf(main_camera_and_eeprom, DEV_INFO_LEN, "%s_%s", 
					sensor_name, "DaLing");		
		}
		
		if(rc < 0){
			store_main_camera_info(sensor_name);
		}else{
			store_main_camera_info(main_camera_and_eeprom);
		}
	}

#endif
}

EXPORT_SYMBOL_GPL(store_camera_info);

static void __exit dev_info_exit(void)
{
	kobject_put(dev_info);
}

module_init(dev_info_init);
module_exit(dev_info_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("tinno");



#include <linux/tinno_project_info.h>

#define MAX_PROJECT_NAME_LENGTH 64
static char tinno_project_name[MAX_PROJECT_NAME_LENGTH+1] = {"Snapdragon"};
static char tinno_market_area[MAX_PROJECT_NAME_LENGTH+1] = {"trunk"};
static char tinno_sensor_info[MAX_PROJECT_NAME_LENGTH+1] = {"trunk"};
char * tinno_get_project_name(void)
{
        int rc;
        const char *temp;
        struct device_node *root;

        if(strcmp(tinno_project_name, "Snapdragon") == 0)
        {
        	root = of_find_node_by_path("/");
        	if (!root){
        		printk("Unable to find device node / \n");
        		goto err_handle;
        	}

        	rc = of_property_read_string(root, TINNO_PROJECT_PROPTITY_NAME, &temp);
        	if (rc && (rc != -EINVAL)) {
        		printk("Unable to read project name\n");
        		goto err_handle;
        	}

        	for(rc = 0; temp[rc] != '\0'; rc++){
                   if (strlen(temp) > MAX_PROJECT_NAME_LENGTH){
                            printk("qcom,tinno-hardware , project name is MAX error!!!\n");
                            break;
                   }
        		if (temp[rc] == '-'){
        			strcpy(tinno_project_name, temp+rc+1);
        			break;
        		}
        	}
        }

err_handle:
	printk("=================project name is %s\n", tinno_project_name);

	return tinno_project_name;
}
//EXPORT_SYMBOL_GPL(tinno_get_project_name);


extern char* saved_command_line;
char * tinno_get_market_area(void)
{
    char * p;
    int length,i;
    
    if(strcmp(tinno_market_area, "trunk") == 0)  
    {
        p = strstr(saved_command_line, "androidboot.Market_Area=");
        if( p == NULL)
        {
            printk(KERN_ERR "cmdline can not find tinno dev config !\n");
            goto fail;
        }

        length = strlen("androidboot.Market_Area=");

        for(i = 0; i < MAX_PROJECT_NAME_LENGTH; i++)
        {
             if((*(p + length + i) == ' ') || (*(p + length + i) == '\0'))
             {
                break;
             }
             tinno_market_area[i] = *(p + length + i);
        }
    }

fail:
        return tinno_market_area;
}


char * tinno_get_sensor_info(void)        //if not MBA ,default is NULL, should verify NULL
{
    char * p;
    int length,i;
    
    if(strcmp(tinno_sensor_info, "trunk") == 0)  
    {
        p = strstr(saved_command_line, "androidboot.Sensor_Info=");
        if( p == NULL)
        {
            printk(KERN_ERR "cmdline can not find tinno dev config !\n");
            return NULL;
        }

        length = strlen("androidboot.Sensor_Info=");
        
        for(i = 0; i < MAX_PROJECT_NAME_LENGTH; i++)
        {
             if((*(p + length + i) == ' ') || (*(p + length + i) == '\0'))
             {
                break;
             }
             tinno_sensor_info[i] = *(p + length + i);
        }
    }


    return tinno_sensor_info;
}


/*******************************************************************************
*  Name: tinno_project_is
*  Brief: compare the given name with the global tinno board info
*  Input: project_name, the given name to compare with the global tinno board info
*  Output: 
*  Return: return true when match, return false otherwise
*******************************************************************************/
bool tinno_project_is(char *project_name)
{
	if(!strcmp(tinno_get_project_name(), project_name)){
		return true;
	} else {
		return false;
	}
}
