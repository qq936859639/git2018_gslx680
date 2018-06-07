
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/rtpm_prio.h>
#include "tpd.h"
#include <cust_eint.h>
#if defined(WM002_CUSTOMER_HUALING2_WVGA_11)
#include <linux/wakelock.h>
#endif
#if defined (MT6572)  // wanghe 2013-07-28 for mt6572 platform, undef other platform macro
#undef MT6575
#undef MT6577
#undef MT6589
#endif


#ifdef MT6573
#include <mach/mt6573_boot.h>
#include <mt6573_kpd.h>
#endif
#ifdef MT6575
#include <mt6575_kpd.h>
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif

#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#endif
#define TP_GESTURE_PROC_FILE   "tp_gesture"


#include "mtk_gslX680.h"

#define  KEY_GESTURE_U			KEY_3
#define  KEY_GESTURE_UP			KEY_4
#define  KEY_GESTURE_DOWN		KEY_6
#define  KEY_GESTURE_LEFT		KEY_7 
#define  KEY_GESTURE_RIGHT		KEY_5
#define  KEY_GESTURE_O			KEY_O
#define  KEY_GESTURE_E			KEY_1
#define  KEY_GESTURE_M			KEY_2 
#define  KEY_GESTURE_L			KEY_L
#define  KEY_GESTURE_W			KEY_W
#define  KEY_GESTURE_S			KEY_S 
#define  KEY_GESTURE_V			KEY_V
#define  KEY_GESTURE_Z			KEY_Z
#define  KEY_GESTURE_HOME		KEY_HOMEPAGE

#define TPD_PROXIMITY
/*start:added by zhangwei 2013-08-21-11:41:27  for add ps  __DRV_MODIFY_RECORD__*/

#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/wakelock.h>    //added by xen 20140904
static u8 tpd_proximity_flag = 0; //flag whether start alps
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
static struct wake_lock ps_lock;
//static u8 gsl_psensor_data[8]={0};

static u8 tdp_proximity_enabled = 1;

#endif

static struct mutex gsl_i2c_lock;
#define GSL_DEBUG
// wanghe 2014-03-18 open for esd check. maybe should change .h 0x74 & 0x7c
#if defined (YK858_CUSTOMER_HUALING2_FWVGA_12_50)
#define GSL_MONITOR
#endif

//#define HS_WAKE_KEY

#define GSLX680_NAME	"gslX680_second"
#define GSLX680_ADDR	0x40
#define MAX_FINGERS	  	10
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
#define SMBUS_TRANS_LEN	0x01 //update by liuxiaoliang 0x08  wanghe must use this!!
#define GSL_PAGE_REG		0xf0
#define ADD_I2C_DEVICE_ANDROID_4_0
//#define HIGH_SPEED_I2C
//#define FILTER_POINT   // delete by wanghe 2013-08-27  xiu gai xian xing du for wm001 baicheng
#ifdef FILTER_POINT
#define FILTER_MAX	6
#endif

#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

static int gsl_up_flag = 0;
static int tpd_flag = 0;
static int tpd_halt=0;
static char eint_flag = 0;
extern struct tpd_device *tpd;
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

#ifdef GSL_MONITOR
static struct delayed_work gsl_monitor_work;
static struct workqueue_struct *gsl_monitor_workqueue = NULL;
//#define GSL_TIMER_CHECK_CIRCLE        200
//static u32 gsl_timer_data = 0;
//static volatile int gsl_timer_flag = 0;
static u8 int_1st[4] = {0};
static u8 int_2nd[4] = {0};
static char dac_counter = 0;
static char b0_counter = 0;
static char bc_counter = 0;
static char i2c_lock_flag = 0;
#endif


#ifdef GSL_GESTURE
static int gsl_gesture_flag = 1;
static struct input_dev *gsl_power_idev;
static int gsl_lcd_flag = 0;
static char gsl_gesture_c = 0;
static struct i2c_client *i2c_client;

static volatile int gsl_halt_flag = 0;


extern  int  gsl_obtain_gesture(void);
static void kpd_touchpanel_gesture_handler(int key_code);
extern void gsl_GestureExternInt(unsigned int *model,int len); 

extern void gsl_FunIICRead(unsigned int (*ReadIICInt)(unsigned int *, unsigned int, unsigned int));
#endif

static kal_uint32 id_sign[MAX_CONTACTS+1] = {0};
static unsigned char id_state_flag[MAX_CONTACTS+1] = {0};
static unsigned char id_state_old_flag[MAX_CONTACTS+1] = {0};
static kal_uint16 x_old[MAX_CONTACTS+1] = {0};
static kal_uint16 y_old[MAX_CONTACTS+1] = {0};
static kal_uint16 x_new = 0;
static kal_uint16 y_new = 0;


/*  // wanghe 2013-07-27 move to mtk_gslX680.h for customer
#define TPD_HAVE_BUTTON
#define TPD_KEY_COUNT	4
#define TPD_KEYS		{KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}
//  {button_center_x, button_center_y, button_width, button_height
#define TPD_KEYS_DIM	{{70, 2048, 60, 50},{210, 2048, 60, 50},{340, 2048, 60, 50},{470, 2048, 60, 50}}
*/

static DECLARE_WAIT_QUEUE_HEAD(waiter);
#ifdef MT6575 
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned char eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(unsigned char eintno, kal_bool sens);
extern void mt65xx_eint_registration(unsigned char eintno, kal_bool Dbounce_En,
		kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
		kal_bool auto_umask);
#endif

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)
#endif

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_local_factory[TPD_KEY_COUNT] = TPD_KEYS_FACTORY;  // wanghe 2013-08-27
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#ifdef TPD_PROXIMITY
/******************************************************************************
 * Sysfs attributes
 *******************************************************************************/
/*----------------------------------------------------------------------------*/
static ssize_t tpd_proximity_show(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", tdp_proximity_enabled);
}
/*----------------------------------------------------------------------------*/
static ssize_t tpd_proximity_store(struct device_driver *ddri, const char *buf, size_t count)
{
    int enable;

    if(1 == sscanf(buf, "%d", &enable))
    {
        if(enable == 1)
        {
            tdp_proximity_enabled = 1;
        }
        else 
        {
            tdp_proximity_enabled = 0;
        }
    }
    return count;    
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(tpd_proximity,    0666, tpd_proximity_show, tpd_proximity_store);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *tpd_proximity_attr_list[] = {
    &driver_attr_tpd_proximity,
};

static int tpd_proximity_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(tpd_proximity_attr_list)/sizeof(tpd_proximity_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, tpd_proximity_attr_list[idx])))
        {            
            printk("driver_create_file (%s) = %d\n", tpd_proximity_attr_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}
/*----------------------------------------------------------------------------*/
static int tpd_proximity_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(tpd_proximity_attr_list)/sizeof(tpd_proximity_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) 
    {
        driver_remove_file(driver, tpd_proximity_attr_list[idx]);
    }

    return err;
}

#endif

static void startup_chip(struct i2c_client *client)
{
	char write_buf = 0x00;
#if 0  // delete by wanghe 2014-03-18
	u8 buf[4] = {0x00};
	buf[3] = 0x01;
	buf[2] = 0xfe;
	buf[1] = 0x10;
	buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xf0, sizeof(buf), buf);
	buf[3] = 0x00;
	buf[2] = 0x00;
	buf[1] = 0x00;
	buf[0] = 0x0f;
	i2c_smbus_write_i2c_block_data(client, 0x04, sizeof(buf), buf);
	msleep(20);	
#endif	
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf); 	
#ifdef GSL_NOID_VERSION
	gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
#endif

	msleep(10);		
}

static void reset_chip(struct i2c_client *client)
{
	char write_buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);

	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf[0]); 	
	msleep(10);

	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;
	write_buf[3] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xbc, 4, write_buf); 	
	msleep(10);

#ifdef SET_GSL9XX_1_8V
    	//set vddio 1.8
	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0xfe;
	write_buf[3] = 0x01;
	i2c_smbus_write_i2c_block_data(client,0xf0,4,write_buf);
	write_buf[0] = 0x05; 
	write_buf[1] = 0x00; 
	write_buf[2] = 0x00; 
	write_buf[3] = 0x80; 
	i2c_smbus_write_i2c_block_data(client,0x78,4,write_buf);
	msleep(5);
#endif
}

static void clr_reg(struct i2c_client *client)
{
	char write_buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);

	write_buf[0] = 0x03;  // 0x01;  wanghe change to 0x03 2014-03-18
	i2c_smbus_write_i2c_block_data(client, 0x80, 1, &write_buf[0]); 	
	msleep(5);
	
	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf[0]); 	
	msleep(5);

	write_buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(20);
}

#if 0 // def HIGH_SPEED_I2C  // wanghe 2013-07-27 must use this!!
static u32 gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;
	xfer_msg[0].timing = 400;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags |= I2C_M_RD;
	xfer_msg[1].buf = buf;
	xfer_msg[1].timing = 400;

	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		msleep(5);
	}

	return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;
	xfer_msg[0].timing = 400;

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len;
	struct fw_data *ptr_fw;

	printk("=============gsl_load_fw start==============\n");

	ptr_fw = GSLX680_FW;
	source_len = ARRAY_SIZE(GSLX680_FW);
	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == ptr_fw[source_line].offset)
		{
			fw2buf(cur, &ptr_fw[source_line].val);
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			buf[0] = (u8)ptr_fw[source_line].offset;

			fw2buf(cur, &ptr_fw[source_line].val);
			cur += 4;

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
	    			gsl_write_interface(client, buf[0], buf, cur - buf - 1);
	    			cur = buf + 1;
			}

			send_flag++;
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#else

static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	char buf[SMBUS_TRANS_LEN*4] = {0};
	char reg = 0, send_flag = 1, cur = 0;
	
	unsigned int source_line = 0;


	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < data_len; source_line++) 
	{
		if(1 == SMBUS_TRANS_LEN)
		{
			reg = GSL_DOWNLOAD_DATA[source_line].offset;

			buf[0] = (char)(GSL_DOWNLOAD_DATA[source_line].val & 0x000000ff);
			buf[1] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0x0000ff00) >> 8);
			buf[2] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0x00ff0000) >> 16);
			buf[3] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0xff000000) >> 24);

			i2c_smbus_write_i2c_block_data(client, reg, 4, buf); 	
		}
		else
		{
			/* init page trans, set the page val */
			if (GSL_PAGE_REG == GSL_DOWNLOAD_DATA[source_line].offset)
			{
				buf[0] = (char)(GSL_DOWNLOAD_DATA[source_line].val & 0x000000ff);
				i2c_smbus_write_i2c_block_data(client, GSL_PAGE_REG, 1, &buf[0]); 	
				send_flag = 1;
			}
			else 
			{
				if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
					reg = GSL_DOWNLOAD_DATA[source_line].offset;

				buf[cur + 0] = (char)(GSL_DOWNLOAD_DATA[source_line].val & 0x000000ff);
				buf[cur + 1] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0x0000ff00) >> 8);
				buf[cur + 2] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0x00ff0000) >> 16);
				buf[cur + 3] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0xff000000) >> 24);
				cur += 4;

				if (0 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08)) 
				{
					i2c_smbus_write_i2c_block_data(client, reg, SMBUS_TRANS_LEN*4, buf); 	
					cur = 0;
				}

				send_flag++;

			}
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#endif

static int test_i2c(struct i2c_client *client)
{
	char read_buf = 0;
	char write_buf = 0x12;
	int ret, rc = 1;
	
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if  (ret  < 0)  
    		rc --;
	else
		printk("<<<< wanghe gslX680 I read reg 0xf0 is %x\n", read_buf);

	msleep(2);
	ret = i2c_smbus_write_i2c_block_data( client, 0xf0, 1, &write_buf );
	if(ret  >=  0 )
		printk("<<<< wanghe gslX680 I write reg 0xf0 0x12\n");
	
	msleep(2);
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if(ret <  0 )
		rc --;
	else
		printk("<<<< wanghe gslX680 I read reg 0xf0 is 0x%x\n", read_buf);

	return rc;
}

#ifdef GSL_IDENTY_TP_2338
int gsl_get_min(int a,int b)
{
	return (a<=b) ? a : b;
}

int gsl_reg_compare(unsigned int sample, unsigned int compare)
{
	char tmp1;
	int i,*p;
	int result = 0;
	unsigned int filler = 0x01;
	for (i=0; i<length_reg; i++) {
		if((sample & (filler << i)) && (!(compare & (filler << i))))
				result++;
		else if (!(sample & (filler << i))  && ( (compare & (filler << i)))) 
				result += length_reg;
	}
	return result;
}

static void gsl_identify_tp(struct i2c_client *client)
{
	int i;
	unsigned char addr = 0x00;
	unsigned char write_buf = 0x00;
	unsigned char read_b4[4] = {0x00};
	unsigned char read_b8[4] = {0x00};
	unsigned char read_ac[4] = {0x00};
	unsigned int tmp_b8 = 0x00000000;
	unsigned int tmp_ac = 0x00000000;
	
	for (i = 0; i < 8; i++ ) {
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		msleep(50); 	
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(20);
		
		clr_reg(client);
		reset_chip(client);
		gsl_cfg_index = 0;
		gsl_load_fw(client, 
			gsl_cfg_table[gsl_cfg_index].fw, 
			gsl_cfg_table[gsl_cfg_index].fw_size);
		startup_chip(client);
		msleep(200);
		
		addr = 0xb4;
		i2c_smbus_read_i2c_block_data(client,addr, sizeof(read_b4), read_b4);
		msleep(10);
		i2c_smbus_read_i2c_block_data(client,addr, sizeof(read_b4), read_b4);
		msleep(10);
		if(!read_b4[0]&&!read_b4[1]&&!read_b4[2]&&!read_b4[3]){
			printk("[TP]gsl_identify_tp read 0xb4 = 0x%02x%02x%02x%02x /n", 
				read_b4[3],read_b4[2],read_b4[1],read_b4[0]);
			continue;
		}
		
		addr = 0xb8;
		i2c_smbus_read_i2c_block_data(client,addr, sizeof(read_b8), read_b8);
		msleep(10);
		i2c_smbus_read_i2c_block_data(client,addr, sizeof(read_b8), read_b8);
		msleep(10);
		tmp_b8 = (read_b8[3]<<24)|(read_b8[2]<<16)|(read_b8[1]<<8)|read_b8[0];
		printk("[TP]gsl_identify_tp read 0xb8 = 0x%02x%02x%02x%02x /n", 
			read_b8[3],read_b8[2],read_b8[1],read_b8[0]);
		
		addr = 0xac;
		i2c_smbus_read_i2c_block_data(client,addr, sizeof(read_ac), read_ac);
		msleep(10);
		i2c_smbus_read_i2c_block_data(client,addr, sizeof(read_ac), read_ac);
		msleep(10);
		tmp_ac = (read_ac[3]<<24)|(read_ac[2]<<16)|(read_ac[1]<<8)|read_ac[0];
		printk("[TP]gsl_identify_tp read 0xac = 0x%02x%02x%02x%02x/n", 
			read_ac[3],read_ac[2],read_ac[1],read_ac[0]);
		
		tmp0_result = 0;
		tmp1_result = 0;
		tmp2_result = 0;
		tmp1_result = gsl_reg_compare(FRIST_B8, tmp_b8);
		tmp1_result += gsl_reg_compare(FRIST_AC, tmp_ac);
		tmp2_result = gsl_reg_compare(SECOND_B8, tmp_b8);
		tmp2_result += gsl_reg_compare(SECOND_AC, tmp_ac);
		tmp0_result = gsl_get_min(tmp1_result, tmp2_result);
        printk("tmp0_result=%d,tmp1_result=%d,tmp2_result=%d/n",
			tmp0_result,tmp1_result,tmp2_result);

		if (tmp0_result == tmp1_result){
			printk("[TP]gsl_identify_tp TP change 1st TP./n");
			gsl_cfg_index = 1;	/* 1st tp */
			break;
		} else if (tmp0_result == tmp2_result) {
			printk("[TP]gsl_identify_tp TP change 2nd TP./n");
			gsl_cfg_index = 2;	/* 2nd tp */
			break;
		}
	}

}
#endif
static void init_chip(struct i2c_client *client)
{
	int rc;
	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20); 	
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20); 		
	rc = test_i2c(client);
	if(rc < 0)
	{
		printk("------gslX680 test_i2c error------\n");	
		return;
	}	
	clr_reg(client);
	reset_chip(client);
#ifdef GSL_IDENTY_TP_2338
	gsl_identify_tp(client);
#endif
	gsl_load_fw(client, gsl_cfg_table[gsl_cfg_index].fw, gsl_cfg_table[gsl_cfg_index].fw_size);			
	startup_chip(client);
	reset_chip(client);
	startup_chip(client);		
}


static void check_mem_data(struct i2c_client *client)
{
	char read_buf[4]  = {0};
	
	msleep(30);
	i2c_smbus_read_i2c_block_data(client,0xb0, sizeof(read_buf), read_buf);
	
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		printk("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip(client);
	}
}
#ifdef GSL_GESTURE
static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	ssize_t len=0;
	sprintf(&buf[len],"%s\n","tp gesture is on/off:");
	len += (strlen("tp gesture is on/off:")+1);
	if(gsl_gesture_flag == 1){
		sprintf(&buf[len],"%s\n","  on  ");
		len += (strlen("  on  ")+1);
	}else if(gsl_gesture_flag == 0){
		sprintf(&buf[len],"%s\n","  off  ");
		len += (strlen("  off  ")+1);
	}

	sprintf(&buf[len],"%s\n","tp gesture:");
	len += (strlen("tp gesture:")+1);
	sprintf(&buf[len],"%c\n",gsl_gesture_c);
	len += 2;	
    return len;
}
//wuhao start
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	char tmp_buf[16];
	
	if(copy_from_user(tmp_buf, buf, (count>16?16:count))){
		return -1;
	}
	if(buf[0] == '0'){
	//	gsl_gesture_flag = 0;
		printk("[GSL_GESTURE] gsl_sysfs_tpgesturet_store off.\n");
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
		printk("[GSL_GESTURE] gsl_sysfs_tpgesturet_store on.\n");
	}

    return count;
}
static DEVICE_ATTR(tpgesture, S_IRUGO|S_IWUSR, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);
static void gsl_request_power_idev(void)
{
	struct input_dev *idev;
	int rc = 0;
	idev = input_allocate_device();
	if(!idev){
		return;
	}
	gsl_power_idev = idev;
	idev->name = "gsl_gesture";
	idev->id.bustype = BUS_I2C;
	input_set_capability(idev,EV_KEY,KEY_POWER);
	input_set_capability(idev,EV_KEY,KEY_END);
//*

	input_set_capability(idev, EV_KEY, KEY_GESTURE_U); 
	input_set_capability(idev, EV_KEY, KEY_GESTURE_UP); 
	input_set_capability(idev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(idev, EV_KEY, KEY_GESTURE_LEFT); 
	input_set_capability(idev, EV_KEY, KEY_GESTURE_RIGHT); 
	input_set_capability(idev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(idev, EV_KEY, KEY_GESTURE_E); 
	input_set_capability(idev, EV_KEY, KEY_GESTURE_M); 
	input_set_capability(idev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(idev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(idev, EV_KEY, KEY_GESTURE_S); 
	input_set_capability(idev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(idev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(idev, EV_KEY, KEY_GESTURE_HOME);
		
//*
	__set_bit(KEY_GESTURE_RIGHT, idev->keybit);
	__set_bit(KEY_GESTURE_LEFT, idev->keybit);
	__set_bit(KEY_GESTURE_UP, idev->keybit);
	__set_bit(KEY_GESTURE_DOWN, idev->keybit);
	__set_bit(KEY_GESTURE_U, idev->keybit);
	__set_bit(KEY_GESTURE_O, idev->keybit);
	__set_bit(KEY_GESTURE_E, idev->keybit);
	__set_bit(KEY_GESTURE_M, idev->keybit);
	__set_bit(KEY_GESTURE_W, idev->keybit);
	__set_bit(KEY_GESTURE_L, idev->keybit);
	__set_bit(KEY_GESTURE_S, idev->keybit);
	__set_bit(KEY_GESTURE_V, idev->keybit);
	__set_bit(KEY_GESTURE_Z, idev->keybit);
	__set_bit(KEY_GESTURE_HOME, idev->keybit);
//*/
	rc = input_register_device(idev);
	if(rc){
		input_free_device(idev);
		gsl_power_idev = NULL;
	}
}
static unsigned int gsl_gesture_init(void)
{
	int ret;
	struct kobject *gsl_debug_kobj;
	gsl_debug_kobj = kobject_create_and_add("gsl_gesture", NULL) ;
	if (gsl_debug_kobj == NULL)
	{
		printk("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}
	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_tpgesture.attr);
    if (ret)
    {
        printk("%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }
	gsl_request_power_idev();
	printk("[GSL_GESTURE] gsl_gesture_init success.\n");
	return 1;
}

#endif

#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
    if(ch>='0' && ch<='9')
        return (ch-'0');
    else
        return (ch-'a'+10);
}

static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *ptr = page;
	char temp_data[5] = {0};
	unsigned int tmp=0;
	int ret;
	
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_NOID_VERSION
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		ptr += sprintf(ptr,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_NOID_VERSION 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			ptr +=sprintf(ptr,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<512)
			{
					ptr +=sprintf(ptr,"%d\n",gsl_cfg_table[gsl_cfg_index].data_id[tmp]); 
			}
#endif
		}
		else 
		{
			i2c_smbus_write_i2c_block_data(i2c_client,0Xf0,4,&gsl_data_proc[4]);
			ret = i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			if(ret<0)
				i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);

			ptr +=sprintf(ptr,"offset : {0x%02x,0x",gsl_data_proc[0]);
			ptr +=sprintf(ptr,"%02x",temp_data[3]);
			ptr +=sprintf(ptr,"%02x",temp_data[2]);
			ptr +=sprintf(ptr,"%02x",temp_data[1]);
			ptr +=sprintf(ptr,"%02x};\n",temp_data[0]);
		}
	}
	*eof = 1;
	return (ptr - page);
}
static int gsl_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%ld]\n", CONFIG_LEN, count);
        return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
	}	
	//if(copy_from_user(path_buf, buffer, (count<CONFIG_LEN?count:CONFIG_LEN)))
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
#ifdef GSL_MONITOR
	cancel_delayed_work_sync(&gsl_monitor_work);
	i2c_lock_flag = 1;
#endif
		gsl_proc_flag = 1;
		reset_chip(i2c_client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		reset_chip(i2c_client);
		startup_chip(i2c_client);
		gsl_proc_flag = 0;
#ifdef GSL_MONITOR
	i2c_lock_flag = 0;
#endif
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		i2c_smbus_write_i2c_block_data(i2c_client,buf[4],4,buf);
	}
#ifdef GSL_NOID_VERSION
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<512)
		{
			gsl_cfg_table[gsl_cfg_index].data_id[tmp1] = tmp;
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
#endif

#ifdef TPD_PROXIMITY
/*
static void gsl_gain_psensor_data(struct i2c_client *client)
{
	int tmp = 0;
	u8 buf[4]={0};
	
	buf[0]=0x3;
	i2c_smbus_write_i2c_block_data(client,0xf0,4,buf);
	tmp = i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[0]);
	if(tmp <= 0)
	{
		 i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[0]);
	}
	

	buf[0]=0x4;
	i2c_smbus_write_i2c_block_data(client,0xf0,4,buf);
	tmp = i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[4]);
	if(tmp <= 0)
	{
		i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[4]);
	}

	
}
*/
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}
static int tpd_enable_ps(int enable)
{
	u8 buf[4]={0};
	if (enable) {
		//reset_chip(i2c_client);
		wake_lock(&ps_lock);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x2;
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);
		tpd_proximity_flag = 1;
		//add alps of function
		printk("tpd-ps function is on\n");
	}
	else 
	{
		tpd_proximity_flag = 0;
		wake_unlock(&ps_lock);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;

		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x00;
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);	
		printk("tpd-ps function is off\n");
	}
	return 0;
}

static int tpd_ps_operate_gslX680(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if((tpd_enable_ps(1) != 0))
					{
						printk("enable ps fail: %d\n", err);
						return -1;
					}
				//					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						printk("disable ps fail: %d\n", err);
						return -1;
					}
				//					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				printk("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;

				sensor_data->values[0] = tpd_get_ps_value();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			printk("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;

}
#endif
#if 0 // def GSL_MONITOR  // delete by wanghe 2014-03-18
static void gsl_monitor_worker(struct work_struct *work)
{
	u8 buf[4] = {0};
	u32 tmp;
	static int timer_count;
	if(1==tpd_halt){
		return;
	}

	i2c_smbus_read_i2c_block_data(i2c_client, 0xb4, 4,buf);
	tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);

	print_info("[pre] 0xb4 = %x \n",gsl_timer_data);
	print_info("[cur] 0xb4 = %x \n",tmp);
	print_info("gsl_timer_flag=%d\n",gsl_timer_flag);
	if(0 == gsl_timer_flag)
	{
		if(tmp==gsl_timer_data)
		{
			gsl_timer_flag = 1;
			if(0==tpd_halt)
			{
				queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 25);
			}
		}
		else
		{
			gsl_timer_flag = 0;
			timer_count = 0;
			if(0 == tpd_halt)
			{
				queue_delayed_work(gsl_monitor_workqueue, 
					&gsl_monitor_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
	}
	else if(1==gsl_timer_flag){
		if(tmp==gsl_timer_data)
		{
			if(0==tpd_halt)
			{
				timer_count++;
				gsl_timer_flag = 2;
				init_chip(i2c_client);
				gsl_timer_flag = 1;
			}
			if(0 == tpd_halt && timer_count < 20)
			{
				queue_delayed_work(gsl_monitor_workqueue, 
					&gsl_monitor_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		else{
			timer_count = 0;
			if(0 == tpd_halt && timer_count < 20)
			{
				queue_delayed_work(gsl_monitor_workqueue, 
					&gsl_monitor_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		gsl_timer_flag = 0;
	}
	gsl_timer_data = tmp;

}
#endif

#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;
	u16 filter_step_x = 0, filter_step_y = 0;
	
	id_sign[id] = id_sign[id] + 1;
	if(id_sign[id] == 1)
	{
		x_old[id] = x;
		y_old[id] = y;
	}
	
	x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
	y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);

	if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
	{
		filter_step_x = x_err;
		filter_step_y = y_err;
	}
	else
	{
		if(x_err > FILTER_MAX)
			filter_step_x = x_err; 
		if(y_err> FILTER_MAX)
			filter_step_y = y_err;
	}

	if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
	{
		filter_step_x >>= 2; 
		filter_step_y >>= 2;
	}
	else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
	{
		filter_step_x >>= 1; 
		filter_step_y >>= 1;
	}	
	else if(x_err <= 4*FILTER_MAX && y_err <= 4*FILTER_MAX)
	{
		filter_step_x = filter_step_x*3/4; 
		filter_step_y = filter_step_y*3/4;
	}	
	
	x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
	y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);

	x_old[id] = x_new;
	y_old[id] = y_new;
}
#else

static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;

	id_sign[id]=id_sign[id]+1;
	
	if(id_sign[id]==1){
		x_old[id]=x;
		y_old[id]=y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;
		
	if(x>x_old[id]){
		x_err=x -x_old[id];
	}
	else{
		x_err=x_old[id]-x;
	}

	if(y>y_old[id]){
		y_err=y -y_old[id];
	}
	else{
		y_err=y_old[id]-y;
	}

	if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
		x_new = x;     x_old[id] = x;
		y_new = y;     y_old[id] = y;
	}
	else{
		if(x_err > 3){
			x_new = x;     x_old[id] = x;
		}
		else
			x_new = x_old[id];
		if(y_err> 3){
			y_new = y;     y_old[id] = y;
		}
		else
			y_new = y_old[id];
	}

	if(id_sign[id]==1){
		x_new= x_old[id];
		y_new= y_old[id];
	}
	
}
#endif

void tpd_down( int id, int x, int y, int p) 
{
	print_info("------tpd_down id: %d, x:%d, y:%d------ \n", id, x, y);

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id); 	
	input_mt_sync(tpd->dev);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
	#ifdef TPD_HAVE_BUTTON 
		tpd_button(x, y, 1);  
	#endif
	}
	#if 0
	if(y > SCREEN_MAX_Y) //virtual key debounce to avoid android ANR issue
	{
		 msleep(50);
		 print_info("D virtual key \n");
	}
	#endif	
}

void tpd_up(void) 
{
	print_info("------tpd_up------ \n");

	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
	#ifdef TPD_HAVE_BUTTON 
	    tpd_button(0, 0, 0);  
	#endif
	}
}

#ifdef GSL_GESTURE
static void kpd_touchpanel_gesture_handler(int key_code){
	printk("[GSL_GESTURE] input report KEY_CODE = %x \n",key_code);

	input_report_key(gsl_power_idev,key_code,1);
	input_sync(gsl_power_idev);
	input_report_key(gsl_power_idev,key_code,0);
	input_sync(gsl_power_idev);

}
#endif

static void report_data_handle(void)
{
	char touch_data[MAX_FINGERS * 4 + 4] = {0};
	char buf[4] = {0};
	char id, point_num = 0;
	unsigned int x, y, temp_a, temp_b, i,tmp1;
#ifdef GSL_NOID_VERSION
	struct gsl_touch_info cinfo={0};
#endif
		
#ifdef TPD_PROXIMITY
		int err;
		hwm_sensor_data sensor_data;
		if (tpd_proximity_flag == 1)
		{
			i2c_smbus_read_i2c_block_data(i2c_client,0xac,4,buf);
			print_info("gslX680   buf[0] = %d buf[1] = %d,  buf[2] = %d  buf[3] = %d \n",buf[0],buf[1],buf[2],buf[3]);
			
			if (buf[0] == 1 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
			{
				tpd_proximity_detect = 0;
			}
			else
			{
				tpd_proximity_detect = 1;
			}
			print_info("gslX680    ps change   tpd_proximity_detect = %d  \n",tpd_proximity_detect);
			//map and store data to hwm_sensor_data
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			//let up layer to know
			if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			{
				print_info("call hwmsen_get_interrupt_data fail = %d\n", err);
			}
		}
#endif

#ifdef GSL_MONITOR
	//if(2==gsl_timer_flag){
	//	return;
	//}
	if(i2c_lock_flag != 0)
	{
		print_info("<<<< wanghe %s() %d\n", __func__, __LINE__);
		return;
	}
	else
		i2c_lock_flag = 1;
#endif


#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
    {
    	// printk("<<<< wanghe gsl_proc_flag == 1");
        goto schedule;
    }
#endif

	i2c_smbus_read_i2c_block_data(i2c_client, 0x80, 8, &touch_data[0]);
	point_num = touch_data[0];
	if(point_num > 1)
	{
		i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 8, &touch_data[8]);
	}
	if(point_num > 3)
	{
		i2c_smbus_read_i2c_block_data(i2c_client, 0x90, 8, &touch_data[16]);
	}
#ifdef GSL_NOID_VERSION
	if(point_num > 5)
	{
		i2c_smbus_read_i2c_block_data(i2c_client, 0x98, 8, &touch_data[24]);
	}	
	if(point_num > 7)
	{
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa0, 8, &touch_data[32]);
	}
	if(point_num > 9)
	{
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa8, 4, &touch_data[40]);
	}
	
	cinfo.finger_num = point_num;
	print_info("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(i = 0; i < (point_num < MAX_CONTACTS ? point_num : MAX_CONTACTS); i ++)
	{
		temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
		temp_b = touch_data[(i + 1) * 4 + 2];
		cinfo.x[i] = temp_a << 8 |temp_b;
		temp_a = touch_data[(i + 1) * 4 + 1];
		temp_b = touch_data[(i + 1) * 4 + 0];
		cinfo.y[i] = temp_a << 8 |temp_b;		
		print_info("tp-gsl  x = %d y = %d \n",cinfo.x[i],cinfo.y[i]);
	}
	cinfo.finger_num = (touch_data[3]<<24)|(touch_data[2]<<16)|
		(touch_data[1]<<8)|touch_data[0];
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
	}
	point_num = cinfo.finger_num;
#endif

#ifdef GSL_GESTURE
	//printk("[GSL_GESTURE] tpd_halt =%d, gsl_gesture_flag = %d \n",gsl_halt_flag, gsl_gesture_flag);
	if((gsl_halt_flag == 1)&&(gsl_gesture_flag == 1)){
		int ges_key = 0;
		int tmp_c;
		tmp_c = gsl_obtain_gesture();
		printk("[GSL_GESTURE] tmp_c =%d \n",tmp_c);
		
		switch(tmp_c){
			/*
			case (int)'A':
			case (int)'B':
			case (int)'C':
			case (int)'D':
			case (int)'E':
			case (int)'G':
			case (int)'H':
			case (int)'M':
			case (int)'O':
			case (int)'Q':
			case (int)'S':
			case (int)'V':
			case (int)'W':
			case (int)'Y':
			case (int)'Z':
			case (int)'*': //double click
			case 0xa1fa: //right
			case 0xa1fb: //left
			case 0xa1fc: //up
			case 0xa1fd: //down
			case 0x30: //1st key single click
			case 0x38: //3rd key single click
			*/
			case 0x34: //home single click 2nd key
				gsl_gesture_c = (char)tmp_c;
				if(gsl_lcd_flag == 0){
					/*
					printk("[GSL_GESTURE] input report KEY_POWER\n");
					input_report_key(gsl_power_idev,KEY_POWER,1);
					input_sync(gsl_power_idev);
					input_report_key(gsl_power_idev,KEY_POWER,0);
					input_sync(gsl_power_idev);
					*/
				}
				//printk("[GSL_GESTURE] set gsl_lcd_flag = 1\n");
				gsl_lcd_flag = 1;			
				switch(tmp_c){
					/*
					case (int)'C':
						ges_key = 0x34;
						break;
					case (int)'E':
						ges_key = KEY_1;
						break;
					case (int)'M':
						ges_key = KEY_2;
						break;
					case (int)'O':
						ges_key = KEY_O;
						break;
					case (int)'S':
						ges_key = KEY_S;
						break;
					case (int)'V':
						ges_key = KEY_V;
						break;
					case (int)'W':
						ges_key = KEY_W;
						break;
					case (int)'Z':
						ges_key = KEY_Z;
						break;
					case (int)'*': //double click
						ges_key = KEY_3;
						break;
					case 0xa1fa: //right
						ges_key = KEY_5;
						break;
					case 0xa1fb: //left
						ges_key = KEY_7;
						break;
					case 0xa1fc: //up
						ges_key = KEY_4;
						break;
					case 0xa1fd: //down
						ges_key = KEY_6;
						break;
					case 0x30: //1st key single click
						break;
					case 0x38: //3rd key single click
						break;
						*/
					case 0x34: //home single click 2nd key
						ges_key = KEY_GESTURE_HOME;
						break;
					default:
						break;
				}
				if(tmp_c>0)
					kpd_touchpanel_gesture_handler(ges_key);
				break;
			default:
				break;
		}
		//return;
        goto schedule;
	}
#endif

	for(i = 1 ;i <= MAX_CONTACTS; i ++)
	{
		if(point_num == 0)
			id_sign[i] = 0;	
		id_state_flag[i] = 0;
	}
	for(i = 0; i < (point_num < MAX_FINGERS ? point_num : MAX_FINGERS); i ++)
	{
		gsl_up_flag = 0;
	#ifdef GSL_NOID_VERSION
		id = cinfo.id[i];
		x =  cinfo.x[i];
		y =  cinfo.y[i];
	#else
		id = touch_data[(i + 1) * 4 + 3] >> 4;
		temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
		temp_b = touch_data[(i + 1) * 4 + 2];
		x = temp_a << 8 |temp_b;
		temp_a = touch_data[(i + 1) * 4 + 1];
		temp_b = touch_data[(i + 1) * 4 + 0];
		y = temp_a << 8 |temp_b;	
	#endif
	
		if(1 <= id && id <= MAX_CONTACTS)
		{
		#ifdef FILTER_POINT
			filter_point(x, y ,id);
		#else
			record_point(x, y , id);
		#endif

#if 0   //added by xen for test 20140410
        x_new = 480 - x_new;
        y_new = (y_new*854)/800;
        //y_new = (y_new)
#endif
		
			tpd_down(id, x_new, y_new, 10);
			id_state_flag[id] = 1;
		}
	}
	for(i = 1; i <= MAX_CONTACTS; i ++)
	{	
		if( (0 == point_num) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
		{
			id_sign[i]=0;
		}
		id_state_old_flag[i] = id_state_flag[i];
	}			
	if(0 == point_num)
	{
		if(1==gsl_up_flag)
		{
			// printk("<<<< wanghe 1==gsl_up_flag");
			goto schedule;
		}
		gsl_up_flag = 1;
		tpd_up();
	}
	input_sync(tpd->dev);
schedule:
#ifdef GSL_MONITOR
	i2c_lock_flag = 0;
#endif
	return;
}

#ifdef GSL_MONITOR
static void gsl_monitor_worker(void)
{
	u8 write_buf[4] = {0};
	u8 read_buf[4]  = {0};
	char init_chip_flag = 0;
	
	print_info("----------------<<<< wanghe gsl_monitor_worker start -----------------\n");	

	if(i2c_lock_flag != 0)
		goto queue_monitor_work;
	else
		i2c_lock_flag = 1;
	
	// print_info("----------------<<<< wanghe gsl_monitor_worker i2c_lock_flag to 1 -----------------\n");	

	i2c_smbus_read_i2c_block_data(i2c_client, 0xb0, 4, read_buf);
	if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter ++;
	else
		b0_counter = 0;

	if(b0_counter > 1)
	{
		printk("======read 0xb0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		b0_counter = 0;
		goto queue_monitor_init_chip;
	}

	i2c_smbus_read_i2c_block_data(i2c_client, 0xb4, 4, read_buf);	
	
	int_2nd[3] = int_1st[3];
	int_2nd[2] = int_1st[2];
	int_2nd[1] = int_1st[1];
	int_2nd[0] = int_1st[0];
	int_1st[3] = read_buf[3];
	int_1st[2] = read_buf[2];
	int_1st[1] = read_buf[1];
	int_1st[0] = read_buf[0];

	if (int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0]) 
	{
		printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",int_1st[3], int_1st[2], int_1st[1], int_1st[0], int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
		init_chip_flag = 1;
		goto queue_monitor_init_chip;
	}
#if 1 //version 1.4.0 or later than 1.4.0 read 0xbc for esd checking
	i2c_smbus_read_i2c_block_data(i2c_client, 0xbc, 4, read_buf);
	if(read_buf[3] != 0 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
		bc_counter++;
	else
		bc_counter = 0;
	if(bc_counter > 1)
	{
		printk("<<<< wanghe ======read 0xbc: %x %x %x %x======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;    // wanghe 2014-03-18 entry here to reset.. mtk_gsl1688_huling2_12_.h 0x74 & 0x7c  !!!!
		bc_counter = 0;
	}
#else
	write_buf[3] = 0x01;
	write_buf[2] = 0xfe;
	write_buf[1] = 0x10;
	write_buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, write_buf);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 4, read_buf);
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 4, read_buf);
	
	if(read_buf[3] < 10 && read_buf[2] < 10 && read_buf[1] < 10 && read_buf[0] < 10)
		dac_counter ++;
	else
		dac_counter = 0;

	if(dac_counter > 1) 
	{
		printk("======read DAC1_0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		dac_counter = 0;
	}
#endif
queue_monitor_init_chip:
	if(init_chip_flag)
		init_chip(i2c_client);
	
	i2c_lock_flag = 0;
	// print_info("----------------<<<< wanghe gsl_monitor_worker i2c_lock_flag to 0 -----------------\n");	

queue_monitor_work:	
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 100);
}
#endif

static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
	
	do
	{
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		print_info("===touch_event_handler, task running===\n");

		eint_flag = 0;
		report_data_handle();
		
	} while (!kthread_should_stop());
	
	return 0;
}

void tpd_eint_interrupt_handler(void)
{
	print_info("===tpd irq interrupt===\n");

	eint_flag = 1;
	tpd_flag=1; 
	wake_up_interruptible(&waiter);
}

#define GSL_COMPATIBLE    // wanghe 2013-07-30 must add this

#ifdef GSL_COMPATIBLE
static int gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4];
	int i,err=-1;
	for(i=0;i<3;i++)
	{
		err = i2c_smbus_read_i2c_block_data(client,0xfc,4,buf);
		printk("[tp-gsl] 0xfc = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],
			buf[1],buf[0]);
		if(err>0)
		{
			break;
		}
	}
	return err;	
}
#endif
static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
	strcpy(info->type, TPD_DEVICE);
	return 0;
}
/*****************************************************************************
Prototype    	: gsl_read_interface
Description  	: gsl chip tranfer function
Input        	: struct i2c_client *client
u8 reg
u8 *buf
u32 num
Output		: None
Return Value 	: static

 *****************************************************************************/
static int gsl_read_interface(struct i2c_client *client,
        u8 reg, u8 *buf, u32 num)
{
	int err = 0;
	int i;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		temp = (reg + 8) & 0x5c;
		i2c_master_send(client,&temp,1);	
		err = i2c_master_recv(client,&buf[0],4);
		temp = reg;
		i2c_master_send(client,&temp,1);	
		err = i2c_master_recv(client,&buf[0],4);
	}
	
	for(i=0;i<num;)
	{	
		temp = reg + i;
		i2c_master_send(client,&temp,1);
		if((i+8)<num)
			err = i2c_master_recv(client,(buf+i),8);
		else
			err = i2c_master_recv(client,(buf+i),(num-i));
		i+=8;
	}
	mutex_unlock(&gsl_i2c_lock);

	return err;
}

/*****************************************************************************
Prototype    : gsl_write_interface
Description  : gsl chip tranfer function
Input        : struct i2c_client *client
const u8 reg
u8 *buf
u32 num
Output       : None
Return Value : static

 *****************************************************************************/
static int gsl_write_interface(struct i2c_client *client,
        const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1] = {0};
	int err;
	u8 tmp_buf[num + 1];

	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	xfer_msg[0].timing = 400;
	mutex_lock(&gsl_i2c_lock);

	err = i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);

	
	return err;
}

#ifdef GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i;
	print_info("tp-gsl-gesture %s\n",__func__);

	for(i=0;i<len/2;i++){
		buf[0] = ((addr+i*8)/0x80)&0xff;
		buf[1] = (((addr+i*8)/0x80)>>8)&0xff;
		buf[2] = (((addr+i*8)/0x80)>>16)&0xff;
		buf[3] = (((addr+i*8)/0x80)>>24)&0xff;
		gsl_write_interface(i2c_client,0xf0,buf,4);
		gsl_read_interface(i2c_client,(addr+i*8)%0x80,(char *)&data[i*2],8);

	/*	i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		i2c_smbus_read_i2c_block_data(i2c_client,(addr+i*8)%0x80,8,(char *)&data[i*2]);
		i2c_smbus_read_i2c_block_data(i2c_client,(addr+i*8)%0x80,8,(char *)&data[i*2]);
	*/
	}
	if(len%2){
		buf[0] = ((addr+len*4 - 4)/0x80)&0xff;
		buf[1] = (((addr+len*4 - 4)/0x80)>>8)&0xff;
		buf[2] = (((addr+len*4 - 4)/0x80)>>16)&0xff;
		buf[3] = (((addr+len*4 - 4)/0x80)>>24)&0xff;
		gsl_write_interface(i2c_client,0xf0,buf,4);
		gsl_read_interface(i2c_client,(addr+len*4 - 4)%0x80,(char *)&data[len-1],4);

/*		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		i2c_smbus_read_i2c_block_data(i2c_client,(addr+len*4 - 4)%0x80,4,(char *)&data[len-1]);
		i2c_smbus_read_i2c_block_data(i2c_client,(addr+len*4 - 4)%0x80,4,(char *)&data[len-1]);
*/
	}
	return len;
}
#endif

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int err = 0;
	char buffer[5];
	int status=0;
#ifdef TPD_PROXIMITY
		struct hwmsen_object obj_ps;
#endif

	mutex_init(&gsl_i2c_lock);

	printk("==tpd_i2c_probe==\n");

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(100);

#ifdef MT6577
	#ifdef TPD_POWER_SOURCE_CUSTOM
		//power on, need confirm with SA
		hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
	#else
		hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
		hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");   
	#endif
#endif	
#ifdef MT6575
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");      
#endif	
#ifdef MT6573
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
  	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif

#if 1	  // 72
	//power on, need confirm with SA
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "TP");
#endif

	msleep(100);

	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	msleep(50);
#ifdef GSL_COMPATIBLE
	err = gsl_compatible_id(client);
	if(err<0)
	{
		return err;
	}
#endif
	i2c_client = client;	
#ifdef GSL_GESTURE
	gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);

	gsl_FunIICRead(gsl_read_oneframe_data);
#endif
	init_chip(i2c_client);
	check_mem_data(i2c_client);

	  mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	  mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	  mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);	

	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}

#ifdef GSL_MONITOR
	printk( "tpd_i2c_probe () : queue gsl_monitor_workqueue\n");

	INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
	gsl_monitor_workqueue = create_singlethread_workqueue("gsl_monitor_workqueue");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1000);
#endif
#ifdef GSL_GESTURE
    // added by xen for gesture flag, 20141008
    remove_proc_entry(TP_GESTURE_PROC_FILE, NULL);
#endif
#ifdef TPD_PROC_DEBUG
	gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
	//printk("[tp-gsl] [%s] gsl_config_proc = %x \n",__func__,gsl_config_proc);
	if (gsl_config_proc == NULL)
	{
		print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	else
	{
		gsl_config_proc->read_proc = gsl_config_read_proc;
		gsl_config_proc->write_proc = gsl_config_write_proc;
	}
	gsl_proc_flag = 0;
#endif

	printk("==tpd_i2c_probe end==\n");
#ifdef TPD_PROXIMITY
	hwmsen_detach(ID_PROXIMITY);
	obj_ps.self = NULL;
	obj_ps.polling = 0;//interrupt mode
	//obj_ps.polling = 1;//need to confirm what mode is!!!
	obj_ps.sensor_operate = tpd_ps_operate_gslX680;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		printk("attach fail = %d\n", err);
	}		
	//gsl_gain_psensor_data(i2c_client);
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

    if((err = tpd_proximity_create_attr(&client->driver->driver)))
    {
        printk("create attribute err = %d\n", err);
    }
    
#endif

#ifdef GSL_GESTURE
	gsl_gesture_init();
#endif

	return 0;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	printk("==tpd_i2c_remove==\n");
	
	return 0;
}


static const struct i2c_device_id tpd_i2c_id[] = {{TPD_DEVICE,0},{}};
#ifdef ADD_I2C_DEVICE_ANDROID_4_0
static struct i2c_board_info __initdata gslX680_i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (GSLX680_ADDR))};
#else
static unsigned short force[] = {0, (GSLX680_ADDR << 1), I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
static struct i2c_client_address_data addr_data = { .forces = forces,};
#endif

struct i2c_driver tpd_i2c_driver = {
	.driver = {
		// .name = TPD_DEVICE,
	.name = "gslX680",  // changed by wanghe 2013-07-27 for sys/bus/i2c/drivers
	#ifndef ADD_I2C_DEVICE_ANDROID_4_0	 
		.owner = THIS_MODULE,
	#endif
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = tpd_i2c_id,
	.detect = tpd_i2c_detect,
	#ifndef ADD_I2C_DEVICE_ANDROID_4_0
	.address_data = &addr_data,
	#endif
};

int tpd_local_init(void)
{
#if defined(GSL_GESTURE)   // added by xen for gesture flag, 20141013
    struct proc_dir_entry *tp_gesture_proc = NULL;  // added by xen for gesture flag, 20141008
#endif
	printk("==tpd_local_init==\n");

	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	///*
	if(tpd_load_status == 0)
	{
		TPD_DMESG("add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	//*/
#ifdef TPD_HAVE_BUTTON
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())    // wanghe 2013-08-27
     	{
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local_factory, tpd_keys_dim_local);// initialize tpd button data
     	}else{
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
 	}

	// tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);
#endif

#if defined(GSL_GESTURE)   // added by xen for gesture flag, 20141008
    tp_gesture_proc = create_proc_entry(TP_GESTURE_PROC_FILE, 0666, NULL);
    if (tp_gesture_proc == NULL)
		TPD_DMESG("create_proc_entry %s failed\n", TP_GESTURE_PROC_FILE);
#endif

	tpd_type_cap = 1;

	printk("==tpd_local_init end==\n");
	return 0;
}

/* Function to manage low power suspend */
void tpd_suspend(struct early_suspend *h)
{
	printk("==tpd_suspend==\n");

#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		return;
	}
#endif

#ifdef GSL_MONITOR
	printk( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
	cancel_delayed_work_sync(&gsl_monitor_work);
	i2c_lock_flag = 0;  //added by xen 20141108
#endif

#ifdef GSL_GESTURE

	gsl_halt_flag = 1;
	if(gsl_gesture_flag == 1){
		u8 buf[4];
		unsigned int temp;
		gsl_lcd_flag = 0;
		//msleep(1000);   // modified by xen 20141013
		msleep(100); 
		buf[0] = 0xa;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0x1;
		buf[3] = 0x5a;
		i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
		msleep(50);
		return;
	}
#endif

	tpd_halt = 1;

#if 0//def GSL_MONITOR
	printk( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
	cancel_delayed_work_sync(&gsl_monitor_work);
#endif

	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);

}

/* Function to manage power-on resume */
void tpd_resume(struct early_suspend *h)
{
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_enable_ps(1);
		return;
	}
#endif
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
#ifdef GSL_GESTURE
	if(gsl_gesture_flag == 1){
		u8 buf[4];
		unsigned int temp;
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		msleep(20);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(5);
		buf[0] = 0xa;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0x5a;
		i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
		msleep(5);		
	}
	
	//start cty for test
	else
	{
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	}
		gsl_halt_flag = 0;
	//end cty for test	
		
#else
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);	
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

	printk("==tpd_resume==\n");
			
	//start cty for test
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);	
	//end cty for test	
	reset_chip(i2c_client);
	startup_chip(i2c_client);
	check_mem_data(i2c_client);	
/*	
	tpd_halt = 0;
#ifdef GSL_GESTURE	
	gsl_halt_flag = 0;
#endif
*/
#ifdef GSL_MONITOR
	printk( "gsl_ts_resume () : queue gsl_monitor_work\n");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
	i2c_lock_flag = 0;
#endif
	//start cty for test
#if defined(YK850_CUSTOMER_KUCHAO_C0811_FWVGA_50)
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
	tpd_halt = 0;
	//end cty for test	
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = GSLX680_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("Sileadinc gslX680 touch panel driver init\n");
	
#ifdef ADD_I2C_DEVICE_ANDROID_4_0
	//i2c_register_board_info(0, &gslX680_i2c_tpd, 1);
	i2c_register_board_info(1, &gslX680_i2c_tpd, 1);  // changed by wanghe 2013-07-27	
#endif
	if(tpd_driver_add(&tpd_device_driver) < 0)
		printk("add gslX680 driver failed\n");
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
	printk("Sileadinc gslX680 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


