   
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
//#include <cust_eint.h>
//#include <pmic_drv.h>
//#include <cust_i2c.h>

#include <linux/wakelock.h>

#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_boot.h>

//#include "cust_gpio_usage.h"

//custom config
#include "mtk_gslX680_custom_config.h"


//#ifdef TPD_PS_SUPPORT

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
//#endif
#include <mt_boot.h>

//#define GSL_DEBUG
//#define GSL_MONITOR 

#define GSLX680_DEV_NAME	"gslX680"  
#define GSLX680_ADDR	0x40
#define MAX_FINGERS	  	10
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
#define SMBUS_TRANS_LEN	0x01	// VV : 0x01 => 0x08
#define GSL_PAGE_REG		0xf0
//#define ADD_I2C_DEVICE_ANDROID_4_0
// #define HIGH_SPEED_I2C	// VV
// #define FILTER_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	9
#endif

// #define TP_POWER MT65XX_POWER_LDO_VGP2
//#define TP_POWER MT6323_POWER_LDO_VGP2

//#define TPD_PROC_DEBUG
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
#ifdef GSL_IDENTY_TP
static int gsl_tp_type = 0;
static int gsl_identify_tp(struct i2c_client *client);
static unsigned int *gsl_config_data_id;
#endif

static volatile int gsl_halt_flag = 0;

#ifdef GSL_GESTURE

#ifndef GESTURE_FEATURE_DEF_ONOFF
#define GESTURE_FEATURE_DEF_ONOFF 0
#endif
#ifndef CUST_KEY_GES_LEFT
#define CUST_KEY_GES_LEFT		0,0
#endif
#ifndef CUST_KEY_GES_RIGHT
#define CUST_KEY_GES_RIGHT		0,0
#endif
#ifndef CUST_KEY_GES_UP
#define CUST_KEY_GES_UP		    	0,0
#endif
#ifndef CUST_KEY_GES_DOWN
#define CUST_KEY_GES_DOWN		0,0
#endif
#ifndef CUST_KEY_GES_DOUBLECLICK
#define CUST_KEY_GES_DOUBLECLICK	0,0
#endif
#ifndef CUST_KEY_GES_O
#define CUST_KEY_GES_O		    0,0
#endif
#ifndef CUST_KEY_GES_W
#define CUST_KEY_GES_W		    0,0
#endif
#ifndef CUST_KEY_GES_M
#define CUST_KEY_GES_M		    0,0
#endif
#ifndef CUST_KEY_GES_E
#define CUST_KEY_GES_E		    0,0
#endif
#ifndef CUST_KEY_GES_C
#define CUST_KEY_GES_C		    0,0
#endif
#ifndef CUST_KEY_GES_S
#define CUST_KEY_GES_S		    0,0
#endif
#ifndef CUST_KEY_GES_V
#define CUST_KEY_GES_V		    0,0
#endif
#ifndef CUST_KEY_GES_Z
#define CUST_KEY_GES_Z		    0,0
#endif

#if 0//guodm
static int gsl_gesture_flag = 1;
static struct input_dev *gsl_power_idev;
static int gsl_lcd_flag = 0;
static char gsl_gesture_c = 0;
#endif

#endif
 int BOOT_Animation;
static int tpd_flag = 0;
static int tpd_halt=0;
static char eint_flag = 0;
extern struct tpd_device *tpd;
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;
#ifdef GSL_MONITOR
static struct delayed_work gsl_monitor_work;
static struct workqueue_struct *gsl_monitor_workqueue = NULL;
static char int_1st[4] = {0};
static char int_2nd[4] = {0};
static char dac_counter = 0;
static char b0_counter = 0;
static char i2c_lock_flag = 0;
#endif

static int id_sign[MAX_CONTACTS+1] = {0};
static int id_state_flag[MAX_CONTACTS+1] = {0};
static int id_state_old_flag[MAX_CONTACTS+1] = {0};
static int x_old[MAX_CONTACTS+1] = {0};
static int y_old[MAX_CONTACTS+1] = {0};
static int x_new = 0;
static int y_new = 0;


#ifdef TPD_PS_SUPPORT
static unsigned char tpd_ps_flag = 0; //enable--1;disanle--0
static unsigned char tpd_ps_value=1;// 1->far,0->close


static u8 tpd_proximity_flag = 0; //flag whether start alps
static u8 tpd_proximity_working_in_sleep = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
static u8 tpd_proximity_counter = 0;
static u8 tpd_proximity_attached = 1;

static struct wake_lock ps_lock;
static u8 gsl_psensor_data[12]={0};
//zmlin add fro debug
#ifdef GSL_IDENTY_TP_BY_DAC
static u8 tp_identy=0;
static u8 first_init_flag=0; 
#endif
static int tp_init = 0;
//end
static int  roller_touch_count = 0;
static void report_data_handle(void);
#endif

static DECLARE_WAIT_QUEUE_HEAD(waiter);
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
              
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

//#ifdef GSL_IDENTY_TP
//int gsl_identy = -1;
//int gsl_identyarr[10]; 
//module_param(gsl_identy, int, 00644);
//module_param_array(gsl_identyarr, int,NULL ,00644);
//#endif

static unsigned int touch_irq = 0;

/*static void tpd_set_gpio_output(unsigned int GPIO,unsigned int output)
{
   mt_set_gpio_mode(GPIO,GPIO_MODE_00);
   mt_set_gpio_dir(GPIO,GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO,(output>0)? GPIO_OUT_ONE: GPIO_OUT_ZERO);
}*/

#ifdef GSL_GESTURE
//(get_ges_feature_onoff())
#if 1//guodm
static void set_tpd_gesture_key_arr(void)
{
	GE_KEYDATA_T tmp_dat[] = {
		[0] = {(unsigned int)'*',CUST_KEY_GES_DOUBLECLICK},
		[1] = {(unsigned int)'C',CUST_KEY_GES_C},
		[2] = {(unsigned int)'M',CUST_KEY_GES_M},
		[3] = {(unsigned int)'E',CUST_KEY_GES_E},
		[4] = {(unsigned int)'O',CUST_KEY_GES_O},
		[5] = {(unsigned int)'W',CUST_KEY_GES_W},
		[6] = {(unsigned int)'Z',CUST_KEY_GES_Z},
		[7] = {(unsigned int)'V',CUST_KEY_GES_V},
		[8] = {(unsigned int)'S',CUST_KEY_GES_S},

		[9] = {(unsigned int)0xa1fb,CUST_KEY_GES_LEFT},
		[10] = {(unsigned int)0xa1fa,CUST_KEY_GES_RIGHT},
		[11] = {(unsigned int)0xa1fc,CUST_KEY_GES_UP},
		[12] = {(unsigned int)0xa1fd,CUST_KEY_GES_DOWN},
		
	};
	tpd_gesture_key_arr_setting(sizeof(tmp_dat)/sizeof(GE_KEYDATA_T),tmp_dat , GESTURE_FEATURE_DEF_ONOFF);
}
#endif




static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i = 0;
	printk("tp-gsl-gesture %s\n",__func__);

	buf[0] = ((addr+i*4)/0x80)&0xff;
	buf[1] = (((addr+i*4)/0x80)>>8)&0xff;
	buf[2] = (((addr+i*4)/0x80)>>16)&0xff;
	buf[3] = (((addr+i*4)/0x80)>>24)&0xff;
	i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
	i2c_smbus_read_i2c_block_data(i2c_client, (((addr+i*4)%0x80+8)&0x5f), 4, (char *)&data[i]);
	i2c_smbus_read_i2c_block_data(i2c_client, (addr+i*4)%0x80, 4, (char *)&data[i]);
			
	for(i=0;i<len;i++)
	{
			i2c_smbus_read_i2c_block_data(i2c_client, (addr+i*4)%0x80, 4, (char *)&data[i]);
			print_info("data[%d] = 0x%08x\n", i, data[i]);
	}
	
	return len;
}

#endif

static void startup_chip(struct i2c_client *client)
{
	u8 write_buf[4] = {0x00};
#if 0	// VV
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
	i2c_smbus_write_i2c_block_data(client, 0xe0, 4, &write_buf); 	
#ifdef GSL_NOID_VERSION
#ifdef GSL_IDENTY_TP_BY_DAC
	#if	defined(GSL1691_DHS_P03_480X960_HESHENGDA_YUECHENG)
		if(tp_identy==0)
			gsl_DataInit(gsl_config_data_id_0);
	  else  
		  gsl_DataInit(gsl_config_data_id_1);
	#elif defined(GSLX680_DHD_F18_HESHENGDA_CHENGHE_SCALE_TO_LCM)
		if(tp_identy==0)
			gsl_DataInit(gsl_config_data_id_hsd);
	  else  
			gsl_DataInit(gsl_config_data_id);
	#else
		if(tp_identy==0)
			gsl_DataInit(gsl_config_data_id_ch);
	  else  
			gsl_DataInit(gsl_config_data_id);
	#endif
#else
	gsl_DataInit(gsl_config_data_id);
#endif	
#endif

	msleep(10);		
}

static void reset_chip(struct i2c_client *client)
{
	u8 write_buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &(write_buf[0])); 	
	msleep(20);

	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &(write_buf[0])); 	
	msleep(10);

	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;
	write_buf[3] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xbc, 4, write_buf); 	
	msleep(10);
}

static void clr_reg(struct i2c_client *client)
{
	u8 write_buf	= 0;

	write_buf = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf); 	
	msleep(20);

	write_buf = 0x03;	// VV : 0x03 => 0x01
	i2c_smbus_write_i2c_block_data(client, 0x80, 1, &write_buf); 	
	msleep(5);
	
	write_buf = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf); 	
	msleep(5);

	write_buf = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf); 	
	msleep(20);
}

#ifdef HIGH_SPEED_I2C
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
#ifdef GSL_IDENTY_TP_BY_DAC
static void gsl_load_fw_CH(struct i2c_client *client)
{
	char buf[SMBUS_TRANS_LEN*4] = {0};
	char reg = 0, send_flag = 1, cur = 0;
	unsigned int source_line = 0;
	unsigned int source_len = 0;
    struct fw_data *ptr_fw;
	#if defined(GSL1691_DHS_P03_480X960_HESHENGDA_YUECHENG)
		ptr_fw = GSLX680_FW_0;
		source_len = ARRAY_SIZE(GSLX680_FW_0);
	#elif defined(GSLX680_DHD_F18_HESHENGDA_CHENGHE_SCALE_TO_LCM)
		ptr_fw = GSLX680_FW_HSD;
		source_len = ARRAY_SIZE(GSLX680_FW_HSD);
	#else
		ptr_fw = GSLX680_FW_CH;
		source_len = ARRAY_SIZE(GSLX680_FW_CH);
	#endif
	printk("=============gsl_load_fw start==============\n");
	for (source_line = 0; source_line < source_len; source_line++) 
	{
		if(1 == SMBUS_TRANS_LEN)
		{
			reg = ptr_fw[source_line].offset;
			memcpy(&buf[0], &ptr_fw[source_line].val, 4);
			i2c_smbus_write_i2c_block_data(client, reg, 4, buf); 	
		}
		else
		{
			if (GSL_PAGE_REG == ptr_fw[source_line].offset)
			{
				buf[0] = (char)(ptr_fw[source_line].val & 0x000000ff);
				i2c_smbus_write_i2c_block_data(client, GSL_PAGE_REG, 1, &buf[0]); 	
				send_flag = 1;
			}
			else 
			{
				if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
					reg = ptr_fw[source_line].offset;
				memcpy(&buf[cur], &ptr_fw[source_line].val, 4);
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

#ifdef GSL_IDENTY_TP
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
#else
static void gsl_load_fw(struct i2c_client *client)
#endif
{
	char buf[SMBUS_TRANS_LEN*4] = {0};
	char reg = 0, send_flag = 1, cur = 0;
	
	unsigned int source_line = 0;
	unsigned int source_len = 0;
    struct fw_data *ptr_fw;
	
#ifdef GSL_IDENTY_TP
	{
		ptr_fw = GSL_DOWNLOAD_DATA;
		source_len = data_len;
	}
#else
		ptr_fw = GSLX680_FW;
		source_len = ARRAY_SIZE(GSLX680_FW);
#endif
	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		if(1 == SMBUS_TRANS_LEN)
		{
			reg = ptr_fw[source_line].offset;
			memcpy(&buf[0], &ptr_fw[source_line].val, 4);
			i2c_smbus_write_i2c_block_data(client, reg, 4, buf); 	
		}
		else
		{
			/* init page trans, set the page val */
			if (GSL_PAGE_REG == ptr_fw[source_line].offset)
			{
				buf[0] = (char)(ptr_fw[source_line].val & 0x000000ff);
				i2c_smbus_write_i2c_block_data(client, GSL_PAGE_REG, 1, &buf[0]); 	
				send_flag = 1;
			}
			else 
			{
				if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
					reg = ptr_fw[source_line].offset;

				memcpy(&buf[cur], &ptr_fw[source_line].val, 4);
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

static int tpd_read_chip_id(struct i2c_client *client)
{
        u8 id_data[4] = {0};
        u8 chip_id = 0;
        int ret;        
        ret = i2c_smbus_read_i2c_block_data(client,0xfc,4,&id_data[0]);
        if(ret < 0)
        {
           return -1;
        }
        printk("-----success-----chip_id = 0x%x.\n",id_data[2]);
        chip_id = id_data[2];
        return chip_id;
}
static int test_i2c(struct i2c_client *client)
{
	char read_buf = 0;
	char write_buf = 0x12;
	int ret, rc = 1;
	
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if  (ret  < 0)  
    		rc --;
	else
		printk("I read reg 0xf0 is %x\n", read_buf);

	msleep(2);
	ret = i2c_smbus_write_i2c_block_data( client, 0xf0, 1, &write_buf );
	if(ret  >=  0 )
		printk("I write reg 0xf0 0x12\n");
	
	msleep(2);
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if(ret <  0 )
		rc --;
	else
		printk("I read reg 0xf0 is 0x%x\n", read_buf);

	return rc;
}
static int gsl_identify_tp(struct i2c_client *client);

#ifdef GSL_IDENTY_TP_BY_DAC
//#if defined(GSLX680_DHD_F18_HESHENGDA_CHENGHE_SCALE_TO_LCM)
static void gsl_identify_tp_by_dac(struct i2c_client *client)
{
	u8 writebuf[4] = {0};
	u8 readbuf[4] = {0};
	int ret;
	u32 tmp;

    	printk("======gsl_identify_tp_by_dac======\n");
		
	writebuf[0] = 0x00;
	writebuf[1] = 0x10;
	writebuf[2] = 0xfe;
	writebuf[3] = 0x01;
    //i2c_smbus_write_i2c_block_data(client, 0xf0, 4, writebuf);
  #if defined(GSLX680_DHD_F18_HESHENGDA_CHENGHE_SCALE_TO_LCM)
	gsl_ts_write(client, 0xf0, writebuf, 4); 	
    msleep(50);
	//i2c_smbus_read_i2c_block_data(client, 0xf0, 4, readbuf);
	//i2c_smbus_read_i2c_block_data(client, 0xf0, 4, readbuf);
	ret =gsl_ts_read(client,0x18, readbuf, 4);
	if (ret < 0) 
		//i2c_smbus_read_i2c_block_data(client, 0xf0, 4, readbuf);
		gsl_ts_read(client,0x18, readbuf, 4);

	printk("======read 1fe1000,10: %x %x %x %x======\n",readbuf[3], readbuf[2], readbuf[1], readbuf[0]);

	if(first_init_flag ==0 )
    {   
		if(readbuf[3] > 0x15 )
     	{
       	 	tp_identy = 1;
			reset_chip(client);
			gsl_load_fw(client);			
			startup_chip(client);
	  	}
        else
        {
        	tp_identy=0;  
        }
    }
    else if ( tp_identy== 1)
    {
    	reset_chip(client);
		gsl_load_fw(client);
        startup_chip(client);
 	}
    
        first_init_flag =1;

#else
    gsl_ts_write(client, 0xf0, writebuf, 4); 	
    msleep(50);
	//i2c_smbus_read_i2c_block_data(client, 0xf0, 4, readbuf);
	//i2c_smbus_read_i2c_block_data(client, 0xf0, 4, readbuf);
	ret =gsl_ts_read(client,0x10, readbuf, 4);
	if (ret < 0) 
		//i2c_smbus_read_i2c_block_data(client, 0xf0, 4, readbuf);
		gsl_ts_read(client,0x10, readbuf, 4);

	printk("======read 1fe1000,10: %x %x %x %x======\n",readbuf[3], readbuf[2], readbuf[1], readbuf[0]);

	if(first_init_flag ==0 )
    {   
		if(readbuf[0] < 0x15 )
     	{
       	 	tp_identy = 1;
			reset_chip(client);
			gsl_load_fw(client);			
			startup_chip(client);
	  	}
        else
        {
        	tp_identy=0;  
        }
    }
    else if ( tp_identy== 1)
    {
    	reset_chip(client);
		gsl_load_fw(client);
        startup_chip(client);
 	}
    
        first_init_flag =1;

#endif
}
#endif
static void init_chip(struct i2c_client *client)
{
	int rc;
	u8 buf[4];
#ifdef GSL_IDENTY_TP
		u32 tmp;
#endif	
    printk("(whs debug)->[init_chip]:enter.\n");
	rc = test_i2c(client);
	if(rc < 0)
	{
		printk("------gslX680 test_i2c error------\n");	
		return;
	}
	
	clr_reg(client);
	reset_chip(client);
#if 1
#ifdef GSL_IDENTY_TP
		if(0==gsl_tp_type)
			gsl_identify_tp(client);
			msleep(10);
			gsl_identify_tp(client);
#endif	
	reset_chip(client);
	clr_reg(client);
	reset_chip(client);
#ifdef GSL_IDENTY_TP
	#if defined(GSLX680_DHD_E01_QHD_CHENHE_HESHENGDA)\
		||defined(GSLX680_DHD_Y02_QHD_CHENHE_HESHENGDA)\
		||defined(GSLX680_DHD_Y01_QHD_CHENHE_HESHENGDA)
		if(1==gsl_tp_type){ 
			tmp = ARRAY_SIZE(GSLX680_FW_CH); 
			gsl_load_fw(client,GSLX680_FW_CH,tmp); 
		} 
		else if(2==gsl_tp_type){ 
			tmp = ARRAY_SIZE(GSLX680_FW_HSD); 
			gsl_load_fw(client,GSLX680_FW_HSD,tmp); 
		} 
        else { 
            tmp = ARRAY_SIZE(GSLX680_FW_HSD); 
            gsl_load_fw(client,GSLX680_FW_HSD,tmp); 
        } 
  #elif defined(GSLX680_DHD_Y03_QHD_YUECHENG)
		if(1==gsl_tp_type){ 
			tmp = ARRAY_SIZE(GSLX680_FW_3_ch); 
			gsl_load_fw(client,GSLX680_FW_3_ch,tmp); 
		} 
		else if(2==gsl_tp_type){ 
			tmp = ARRAY_SIZE(GSLX680_FW_2_yc); 
			gsl_load_fw(client,GSLX680_FW_2_yc,tmp); 
		}else if(3==gsl_tp_type){ 
			tmp = ARRAY_SIZE(GSLX680_FW_3_hsd); 
			gsl_load_fw(client,GSLX680_FW_3_hsd,tmp); 
		} 
        else { 
            tmp = ARRAY_SIZE(GSLX680_FW_2_yc); 
            gsl_load_fw(client,GSLX680_FW_2_yc,tmp); 
        } 
	#elif defined(GSL1691_LYX_Y400_720X1440_DACHENG_LIHAOJIE)
		if(1==gsl_tp_type){
			tmp = ARRAY_SIZE(GSLX680_FW_LHJ);
			gsl_load_fw(client,GSLX680_FW_LHJ,tmp);
		}
		else if(2==gsl_tp_type){
			tmp = ARRAY_SIZE(GSLX680_FW_DC);   //GSLX680_FW_LHJ   GSLX680_FW_DC
			gsl_load_fw(client,GSLX680_FW_DC,tmp);
		}
               else {
                        tmp = ARRAY_SIZE(GSLX680_FW_LHJ);
                        gsl_load_fw(client,GSLX680_FW_LHJ,tmp);
                }
  #elif defined(GSLX680_CONFIG_DHD_X09_HSD_HST_YS_GES)
		if(1==gsl_tp_type){
			tmp = ARRAY_SIZE(GSLX680_FW_GST);
			gsl_load_fw(client,GSLX680_FW_GST,tmp);
		}
		else if(2==gsl_tp_type){
			tmp = ARRAY_SIZE(GSLX680_FW_ZHZY);
			gsl_load_fw(client,GSLX680_FW_ZHZY,tmp);
		}
             else if(3==gsl_tp_type){
			tmp = ARRAY_SIZE(GSLX680_FW_YUCHENG);
			gsl_load_fw(client,GSLX680_FW_YUCHENG,tmp);
		}
               else {
                        tmp = ARRAY_SIZE(GSLX680_FW_GST);
                        gsl_load_fw(client,GSLX680_FW_GST,tmp);
                }

                
#else
  	tmp = ARRAY_SIZE(GSLX680_FW);
  	gsl_load_fw(client,GSLX680_FW,tmp);         
	#endif
#else
#ifdef GSL_IDENTY_TP_BY_DAC
		gsl_load_fw_CH(client);
#else
	gsl_load_fw(client);
#endif					
#endif	
#endif	
	startup_chip(client);
	msleep(200);
#ifdef GSL_IDENTY_TP_BY_DAC
	gsl_identify_tp_by_dac(client);
#endif	
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


#ifdef  TPD_PS_SUPPORT	//TPD_PROXIMITY
static void gsl_gain_psensor_data(struct i2c_client *client)
{
	int tmp = 0;
	u8 buf[4]={0};
	/**************************/
	buf[0]=0x3;
	i2c_smbus_write_i2c_block_data(client,0xf0,4,buf);
	tmp = i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[0]);
	if(tmp <= 0)
	{
		 i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[0]);
	}
  	/**************************/
	
	buf[0]=0x4;
	i2c_smbus_write_i2c_block_data(client,0xf0,4,buf);
	tmp = i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[4]);
	if(tmp <= 0)
	{
		i2c_smbus_read_i2c_block_data(client,0x0,4,&gsl_psensor_data[4]);
	}
	/**************************/
	
/*	buf[0]=0x6;
	i2c_smbus_write_i2c_block_data(client,0xf0,4,buf);
	tmp = i2c_smbus_read_i2c_block_data(client,0x44,4,&gsl_psensor_data[8]);
	if(tmp <= 0)
	{
		i2c_smbus_read_i2c_block_data(client,0x44,4,&gsl_psensor_data[8]);
	}*/
	/**************************/	
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}
static int tpd_enable_ps(int enable)
{
	u8 buf[4]={0};
	if (enable) {
		wake_lock(&ps_lock);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x3;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4,buf);
		buf[3] = 0x5a;
		buf[2] = 0x5a;
		buf[1] = 0x5a;
		buf[0] = 0x5a;
		i2c_smbus_write_i2c_block_data(i2c_client, 0, 4,buf);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4,buf);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x3;
		i2c_smbus_write_i2c_block_data(i2c_client, 0, 4,buf);
		
/*		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x6;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4,buf);
		buf[3] = 0x0;
		buf[2] = 0x15; //@sensertive low >>>
		buf[1] = 0x0;
		buf[0] = 0x15;	//@sensertive low >>> max 0x1f, min 0x01
		i2c_smbus_write_i2c_block_data(i2c_client, 0x44, 4,buf);*/
				
		tpd_proximity_flag = 1;
		//add alps of function
		printk("tpd-ps function is on\n");
	} else {
		tpd_proximity_flag = 0;
		wake_unlock(&ps_lock);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x3;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4,buf);
		buf[3] = gsl_psensor_data[3];
		buf[2] = gsl_psensor_data[2];
		buf[1] = gsl_psensor_data[1];
		buf[0] = gsl_psensor_data[0];
		printk("tpd-ps off, gsl_psensor_data[3] = 0x%02x,gsl_psensor_data[2] = 0x%02x,gsl_psensor_data[1] = 0x%02x,gsl_psensor_data[0] = 0x%02x,\n",gsl_psensor_data[3],gsl_psensor_data[2],gsl_psensor_data[1],gsl_psensor_data[0]);
		i2c_smbus_write_i2c_block_data(i2c_client, 0, 4,buf);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4,buf);
		buf[3] = gsl_psensor_data[7];
		buf[2] = gsl_psensor_data[6];
		buf[1] = gsl_psensor_data[5];
		buf[0] = gsl_psensor_data[4];
		printk("tpd-ps off, gsl_psensor_data[7] = 0x%02x,gsl_psensor_data[6] = 0x%02x,gsl_psensor_data[5] = 0x%02x,gsl_psensor_data[4] = 0x%02x,\n",gsl_psensor_data[7],gsl_psensor_data[6],gsl_psensor_data[5],gsl_psensor_data[4]);
		i2c_smbus_write_i2c_block_data(i2c_client, 0x0, 4,buf);
/*		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x6;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4,buf);
		buf[3] = gsl_psensor_data[11];
		buf[2] = gsl_psensor_data[10];
		buf[1] = gsl_psensor_data[9];
		buf[0] = gsl_psensor_data[8];
		printk("tpd-ps off, gsl_psensor_data[11] = 0x%02x,gsl_psensor_data[10] = 0x%02x,gsl_psensor_data[9] = 0x%02x,gsl_psensor_data[8] = 0x%02x,\n",gsl_psensor_data[11],gsl_psensor_data[10],gsl_psensor_data[9],gsl_psensor_data[8]);
		i2c_smbus_write_i2c_block_data(i2c_client, 0x44, 4,buf);*/
		printk("tpd-ps function is off\n");
		//startup_chip(i2c_client);
		//msleep(10);
		//check_mem_data(i2c_client);		
	}
	return 0;
}

static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *sensor_data;

	printk("z3__________________ tpd_ps_operate command: %d\n", command);

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
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;

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


#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
    if(ch>='0' && ch<='9')
        return (ch-'0');
    else
        return (ch-'a'+10);
}
#if 0
static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *ptr = page;
	char temp_data[5] = {0};
	unsigned int tmp=0;
	
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
			if(tmp>=0&&tmp<512/*ARRAY_SIZE(gsl_config_data_id)*/)
			{
					ptr +=sprintf(ptr,"%d\n",gsl_config_data_id[tmp]); 
			}
#endif
		}
		else 
		{
			i2c_smbus_write_i2c_block_data(i2c_client,0Xf0,4,&gsl_data_proc[4]);
			if(gsl_data_proc[0] < 0x80)
				i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
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
#else
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_NOID_VERSION
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_NOID_VERSION 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
#ifdef	GSL_IDENTY_TP_BY_DAC
			if(tp_identy == 1)
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			else
				seq_printf(m,"gsl_config_data_id_ch[%d] = ",tmp);
#else
				seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
#endif
			if(tmp>=0&&tmp<512/*ARRAY_SIZE(gsl_config_data_id)*/)
			{
#ifdef	GSL_IDENTY_TP_BY_DAC
	#if defined(GSLX680_DHD_F18_HESHENGDA_CHENGHE_SCALE_TO_LCM)
				if(tp_identy == 1)
					seq_printf(m,"%d\n",gsl_config_data_id[tmp]); 
						else
							seq_printf(m,"%d\n",gsl_config_data_id_hsd[tmp]);
	#else
				if(tp_identy == 1)
						 seq_printf(m,"%d\n",gsl_config_data_id[tmp]);
						else
							seq_printf(m,"%d\n",gsl_config_data_id_ch[tmp]);
  #endif
#else
							seq_printf(m,"%d\n",gsl_config_data_id[tmp]);
#endif
			}
#endif
		}
		else 
		{
			i2c_smbus_write_i2c_block_data(i2c_client,0Xf0,4,&gsl_data_proc[4]);
			if(gsl_data_proc[0] < 0x80)
				i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
	return (0);
}
#endif
static int gsl_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp2 = 0;
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
		gsl_proc_flag = 1;
		reset_chip(i2c_client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		reset_chip(i2c_client);
		startup_chip(i2c_client);
		gsl_proc_flag = 0;
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
		tmp2=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<512/*ARRAY_SIZE(gsl_config_data_id)*/)
		{
#ifdef	GSL_IDENTY_TP_BY_DAC
		#if defined(GSLX680_DHD_F18_HESHENGDA_CHENGHE_SCALE_TO_LCM)
			if(tp_identy == 1)
				gsl_config_data_id[tmp1] = tmp2;
			else
				gsl_config_data_id_hsd[tmp1] = tmp2;
		#else
			if(tp_identy == 1)
				gsl_config_data_id[tmp1] = tmp2;
			else
				gsl_config_data_id_ch[tmp1] = tmp2;
		#endif
#else
			gsl_config_data_id[tmp1] = tmp2;
#endif		
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
#if 1
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif
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
#ifdef GSL_IDENTY_TP	
#define GSL_C		100
#ifdef GSLX680_CONFIG_DHD_X09_COMM
#define GSL_CHIP_1    0xffe07801	////heshengda 
#define GSL_CHIP_2    0xff807c07 //0xff807c07	//h02_fw
#define GSL_CHIP_3    0xff80f801 //0xff80f801//yusheng
//#define GSL_CHIP_4	0x00000000  //FOUR    ffe07801
#elif defined(TPCONFIG_DHD_H02_GSL1688_COMM)
#define GSL_CHIP_1	0xffc07807//zhzy  
#define GSL_CHIP_2	0xff807807//gst
#define GSL_CHIP_3	0xff807c07//yusheng
#elif defined(TPCONFIG_DHD_X10_HSD_YS_GSL1688)
#define GSL_CHIP_1    0x00000000        ////heshengda 
#define GSL_CHIP_2    0xff807807//0xff807c07    //h02_fw
#define GSL_CHIP_3    0xffe07801//0xff80f801//yusheng


#elif defined(GSLX680_CONFIG_DHD_X09_HSD_HST_YS_GES)
#define GSL_CHIP_1    0xffc0f801	//1681f yusheng
#define GSL_CHIP_2    0xffe07801  //1691f hsd
#define GSL_CHIP_3    0xff807c07  //none h02
#elif defined(GSLX680_DHD_Y01_QHD_CHENHE_HESHENGDA)
#define GSL_CHIP_1  0xff807807  //CH 
#define GSL_CHIP_2  0xff807801  ////HSD 
#define GSL_CHIP_3  0xff80f801 //yusheng
#elif defined(GSLX680_DHD_E01_QHD_CHENHE_HESHENGDA)
#define GSL_CHIP_1  0xff807803  ////CH 
#define GSL_CHIP_2  0xff80f801  // HSD
#define GSL_CHIP_3  0xff80f801 //yusheng
#elif defined(GSLX680_DHD_Y02_QHD_CHENHE_HESHENGDA)
#define GSL_CHIP_1  0xff807801  ////CH 
#define GSL_CHIP_2  0xffc07801  // HSD
#define GSL_CHIP_3  0xff80f801 //yusheng
#elif defined(GSLX680_DHD_Y03_QHD_YUECHENG)
#define GSL_CHIP_1  0xff807803   ////3个按键 
#define GSL_CHIP_2  0xff807807  // 2个按键
#define GSL_CHIP_3  0xff80f809   //3个按键-hsd
#elif defined(GSL1691_LYX_Y400_720X1440_DACHENG_LIHAOJIE)
#define GSL_CHIP_1  0xfff00000  ////lihaojie 
#define GSL_CHIP_2  0xfff80000  //dacheng
#define GSL_CHIP_3  0xff80f801 //yusheng
#else
#define GSL_CHIP_1  0xfff00000  ////lihaojie 
#define GSL_CHIP_2  0xfff80000  //dacheng
#define GSL_CHIP_3  0xff80f801 //yusheng
#endif
static unsigned int gsl_count_one(unsigned int flag)
{
	unsigned int tmp=0; 
	int i =0;

	for (i=0 ; i<32 ; i++) {
		if (flag & (0x1 << i)) {
			tmp++;
		}
	}
	return tmp;
}

static int gsl_identify_tp(struct i2c_client *client)
{
	u8 buf[4];
	int i,err=1;
	int flag=0;
	unsigned int tmp,tmp0;
	unsigned int tmp1,tmp2,tmp3,tmp4;
	u32 num;

identify_tp_repeat:
	clr_reg(client);
	reset_chip(client);
	num = ARRAY_SIZE(GSL_TP_CHECK_FW);
	gsl_load_fw(client,GSL_TP_CHECK_FW,num);
	startup_chip(client);
	msleep(200);
	i2c_smbus_read_i2c_block_data(client,0xb4,4,buf);
	print_info("the test 0xb4 = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],buf[1],buf[0]);

	if (((buf[3] << 8) | buf[2]) > 1) {
		print_info("[TP-GSL][%s] is start ok\n",__func__);
		msleep(20);
		i2c_smbus_read_i2c_block_data(client,0xb8,4,buf);
		tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		printk("tong the test 0xb8 = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],buf[1],buf[0]);

		tmp1 = gsl_count_one(GSL_CHIP_1^tmp);
		tmp0 = gsl_count_one((tmp&GSL_CHIP_1)^GSL_CHIP_1); 
		tmp1 += tmp0*GSL_C;  
		print_info("[TP-GSL] tmp1 = %d\n",tmp1); 
		
		tmp2 = gsl_count_one(GSL_CHIP_2^tmp); 
		tmp0 = gsl_count_one((tmp&GSL_CHIP_2)^GSL_CHIP_2); 
		tmp2 += tmp0*GSL_C;
		print_info("[TP-GSL] tmp2 = %d\n",tmp2);
        	
		tmp3 = gsl_count_one(GSL_CHIP_3^tmp);
		tmp0 = gsl_count_one((tmp&GSL_CHIP_3)^GSL_CHIP_3); 
		tmp3 += tmp0*GSL_C;
		print_info("[TP-GSL] tmp3 = %d\n",tmp3);
		
		//tmp4 = gsl_count_one(GSL_CHIP_4^tmp); 
		//tmp0 = gsl_count_one((tmp&GSL_CHIP_4)^GSL_CHIP_4);
		//tmp4 += tmp0*GSL_C;
        
		print_info("[TP-GSL] tmp4 = %d\n",tmp4);
	
		if (0xffffffff == GSL_CHIP_1) {
			tmp1=0xffff;
		}
		if (0xffffffff == GSL_CHIP_2) {
			tmp2=0xffff;
		}
		if (0xffffffff == GSL_CHIP_3) {
			tmp3=0xffff;
		}
		print_info("[TP-GSL] tmp1 = %d\n",tmp1); 
		print_info("[TP-GSL] tmp2 = %d\n",tmp2);
		print_info("[TP-GSL] tmp3 = %d\n",tmp3);
		print_info("[TP-GSL] tmp4 = %d\n",tmp4);
		tmp = tmp1;
		if (tmp1 > tmp2) {
			tmp = tmp2; 
		}
		if(tmp > tmp3)
		{
			tmp = tmp3;
		}
	#if defined(GSLX680_DHD_E01_QHD_CHENHE_HESHENGDA)\
		||defined(GSLX680_DHD_Y02_QHD_CHENHE_HESHENGDA)\
		||defined(GSLX680_DHD_Y01_QHD_CHENHE_HESHENGDA)
		if(tmp == tmp1){ 
			gsl_config_data_id = gsl_config_data_id_CH; 
			gsl_tp_type = 1; 
			printk("E01 tong gsl_tp_type = %d", gsl_tp_type);			 
		} else if(tmp == tmp2) { 
			gsl_config_data_id = gsl_config_data_id_HSD; 
			gsl_tp_type = 2; 
			printk("E01 tong gsl_tp_type = %d", gsl_tp_type);			 
		}
	#elif defined(GSLX680_DHD_Y03_QHD_YUECHENG)
	//	}
		if(tmp == tmp1){ 
			gsl_config_data_id = gsl_config_data_id_3_ch; 
			gsl_tp_type = 1; 
			printk("dhd l20 y03 tong gsl_tp_type = %d", gsl_tp_type);			 
		} else if(tmp == tmp2) { 
			gsl_config_data_id = gsl_config_data_id_2_yc; 
			gsl_tp_type = 2; 
			printk("dhd l20 y03 tong gsl_tp_type = %d", gsl_tp_type);			 
		}else if(tmp == tmp3) { 
			gsl_config_data_id = gsl_config_data_id_3_hsd; 
			gsl_tp_type = 3; 
			printk("dhd l20 y03 tong gsl_tp_type = %d", gsl_tp_type);			 
		}
	#elif defined(GSL1691_LYX_Y400_720X1440_DACHENG_LIHAOJIE)
		if(tmp == tmp1){
			gsl_config_data_id = gsl_config_data_id_LHJ;
			gsl_tp_type = 1;
			printk("tong gsl_tp_type = %d", gsl_tp_type);			
		} else if(tmp == tmp2) {
			gsl_config_data_id = gsl_config_data_id_DC;  //gsl_config_data_id_DC
			gsl_tp_type = 2;
			printk("tong gsl_tp_type = %d", gsl_tp_type);			
		} 
	#elif defined(GSLX680_CONFIG_DHD_X09_HSD_HST_YS_GES)
		if(tmp == tmp1){
			gsl_config_data_id = gsl_config_data_id_gst;
			gsl_tp_type = 1;
	printk("tong gsl_tp_type = %d", gsl_tp_type);			
		} else if(tmp == tmp2) {
			gsl_config_data_id = gsl_config_data_id_zhzy;
			gsl_tp_type = 2;
	printk("tong gsl_tp_type = %d", gsl_tp_type);			
		} 
               else if(tmp == tmp3) {
			gsl_config_data_id = gsl_config_data_id_yucheng;
			gsl_tp_type = 3;
		printk("tong gsl_tp_type = %d", gsl_tp_type);			

		}
	#else
			gsl_config_data_id = gsl_config_data_id;
			gsl_tp_type = 0;
			printk("tong gsl_tp_type = %d", gsl_tp_type);		
	#endif
		err = 1;
	} else {
		flag++;
		if(flag < 3) {
			goto identify_tp_repeat;
		}
                else{
	#if defined(GSLX680_DHD_E01_QHD_CHENHE_HESHENGDA)\
		||defined(GSLX680_DHD_Y02_QHD_CHENHE_HESHENGDA)\
		||defined(GSLX680_DHD_Y01_QHD_CHENHE_HESHENGDA)
		gsl_config_data_id = gsl_config_data_id_HSD;
	#elif defined(GSLX680_DHD_Y03_QHD_YUECHENG)
		gsl_config_data_id = gsl_config_data_id_3_ch; 
	#elif defined(GSL1691_LYX_Y400_720X1440_DACHENG_LIHAOJIE)
		gsl_config_data_id = gsl_config_data_id_LHJ; 
	#elif defined(GSLX680_CONFIG_DHD_X09_HSD_HST_YS_GES)
                      gsl_config_data_id = gsl_config_data_id_gst;
	#else
        gsl_config_data_id = gsl_config_data_id;
	#endif
                    }
		err = 0;
	}
//	gsl_identyarr[0] = tmp;
//	gsl_identyarr[1] = tmp1;
//	gsl_identyarr[2] = tmp2;
//	gsl_identyarr[3] = tmp3;
//	gsl_identyarr[4] = gsl_tp_type;
//	gsl_identy = gsl_tp_type;
	
	//printk("2222222 gsl_tp_type = %d", gsl_tp_type);
	return err; 
}
#endif

#if defined(GSL_SCALE_MAX_WIDTH)&&defined(GSL_SCALE_MAX_HIGHT)
static void tpd_toucharea_scale(int *p_x,int * p_y)
{
	//TPD_RES_X TPD_RES_Y
	//GSL_SCALE_MAX_WIDTH GSL_SCALE_MAX_HIGHT

	if(*p_x < GSL_SCALE_MAX_WIDTH && *p_y < GSL_SCALE_MAX_HIGHT){
		*p_x *= TPD_RES_X;
		*p_x /= GSL_SCALE_MAX_WIDTH;

		*p_y *= TPD_RES_Y;
		*p_y /= GSL_SCALE_MAX_HIGHT;
	}
	//if(*p_y == )
}
#endif


void tpd_down( int id, int x, int y, int p) 
{
	print_info("------tpd_down id: %d, x:%d, y:%d------ \n", id, x, y);

	#if defined(GSL_SCALE_MAX_WIDTH)&&defined(GSL_SCALE_MAX_HIGHT)
	tpd_toucharea_scale(&x,&y);
	#endif

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
	//if(y > SCREEN_MAX_Y) //virtual key debounce to avoid android ANR issue
	//{
	//	 msleep(50);
	//	 print_info("D virtual key \n");
	//}	
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
 
static void report_data_handle(void)
{
	char touch_data[MAX_FINGERS * 4 + 4] = {0};
	char buf[4] = {0};
	char id, point_num = 0;
	unsigned int x, y, temp_a, temp_b, i,tmp1,drv_tempxy;
	printk("=======report_data_handle=====\n");	
#ifdef GSL_MONITOR
	if(i2c_lock_flag != 0)
		return;
	else
		i2c_lock_flag = 1;
#endif
//add for debug
#ifdef TPD_PS_SUPPORT
			int err;
			struct hwm_sensor_data sensor_data;
#endif
//end
#ifdef GSL_NOID_VERSION
	struct gsl_touch_info cinfo={0};
#endif
	
#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
        goto i2c_lock;
#endif
//zmlin add for debug ps
#ifdef TPD_PS_SUPPORT
			/*added by bernard*/
			if (tpd_proximity_flag == 1)//zmlin mod
			{
				//gsl_read_interface(i2c_client,0xac,buf,4);
				//i2c_smbus_read_i2c_block_data(i2c_client,0xb4,buf,4);
				i2c_smbus_read_i2c_block_data(i2c_client,0xac,4,buf);
			printk("\n***========%x,%x,%x,%x\n",buf[0],buf[1],buf[2],buf[3]);
				if (buf[0] == 1 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
				{
					tpd_proximity_detect = 0;
					printk("zmlin %s,%d\n",__func__,__LINE__);
					//sensor_data.values[0] = 0;
				}
				else
				{
					tpd_proximity_detect = 1;
					//sensor_data.values[0] = 1;
				}
				//get raw data
				printk(" ps change\n");
				//map and store data to hwm_sensor_data
			printk("[tp-gsl] [%s] tpd_proximity_detect = %d\n",__func__,tpd_proximity_detect);
				sensor_data.values[0] = tpd_get_ps_value();
				sensor_data.value_divide = 1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
				//let up layer to know
				if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
				{
					printk("call hwmsen_get_interrupt_data fail = %d\n", err);
				}
			}
			/*end of added*/
#endif
//update by leweihua
//add for debug end
	i2c_smbus_read_i2c_block_data(i2c_client, 0x80, 4, &touch_data[0]);
	point_num = touch_data[0];
	if(point_num > 0)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x84, 8, &touch_data[4]);
	if(point_num > 2)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x8c, 8, &touch_data[12]);
	if(point_num > 4 && point_num < MAX_FINGERS)
//add for debug
		i2c_smbus_read_i2c_block_data(i2c_client, 0x94, 8, &touch_data[20]);
	if(point_num > 6 && point_num < MAX_FINGERS)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x9c, 8, &touch_data[28]);
	if(point_num > 8 && point_num < MAX_FINGERS)
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa4, 8, &touch_data[36]);
//add for debug
	/*if(point_num > 4)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x94, 8, &touch_data[20]);
	if(point_num > 6)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x9c, 8, &touch_data[28]);
	if(point_num > 8)
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa4, 8, &touch_data[36]);*/
//end
	
#ifdef GSL_NOID_VERSION
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
#if 1//guodm
	if(get_ges_feature_onoff())
	{
		unsigned int key_code = 0;
		printk("[GSL_GESTURE] tpd_halt =%d \n",tpd_halt); 
		if(tpd_halt == 1){ 
			int tmp_c; 
			tmp_c = gsl_obtain_gesture(); 
			if(tmp_c== (int)0xa1fb||tmp_c== (int)0xa1fa||tmp_c== (int)0xa1fc||tmp_c== (int)0xa1fd||tmp_c== (int)'M'||tmp_c== (int)'S'||tmp_c== (int)'V'||tmp_c== (int)'*'||tmp_c== (int)'C' || tmp_c==(int)'O' || tmp_c==(int)'E' || tmp_c==(int)'W' || tmp_c==(int)'Z'){
			printk("[GSL_GESTURE] tmp_c =0x%x\n",tmp_c); 
			
			if(get_ges_feature_enable_by_tpcode(tmp_c))
				key_code = get_ges_keycode_by_tpcode(tmp_c);

			if(key_code != 0){
				tpd_kernel_key(key_code, 1);
				tpd_kernel_key(key_code, 0);
				printk("[GSL_GESTURE] report key tmp_c = %d\n",tmp_c); 
				printk("[GSL_GESTURE] set tpd_halt = 0\n"); 
				tpd_halt = 0;
			}
			
			}
			
			goto i2c_lock; 
		}
	}
#else
			printk("[GSL_GESTURE] tpd_halt =%d, gsl_gesture_flag = %d \n",tpd_halt,gsl_gesture_flag); 
			if(tpd_halt == 1 && gsl_gesture_flag == 1){ 
				int tmp_c; 
				int i = 0;
				u8 key_data = 0;
				tmp_c = gsl_obtain_gesture(); 
				//print_info("[GSL_GESTURE] tmp_c =%x\n",tmp_c); 
				switch(tmp_c){ 
				case (int)'C':	
					key_data = 'c';
					print_info("[GSL_GESTURE]ccccc tmp_c =%d\n",tmp_c);
					break;
				case (int)'E': 
					 key_data = 'e';	
					 print_info("[GSL_GESTURE]eeeeeeeeee tmp_c =%d\n",tmp_c);
					break;
				case (int)'W': 
					 key_data = 'w';
					 print_info("[GSL_GESTURE]WWWWWWW tmp_c =%d\n",tmp_c);
					break;
				case (int)'O': 
					 key_data = 'o';
					 print_info("[GSL_GESTURE]OOOOO tmp_c =%d\n",tmp_c);
					break;
		//		case (int)'M': 
		//			 key_data = 'm';
		//			 break;
				case (int)'Z': 
					 key_data = 'z';
					 print_info("[GSL_GESTURE]ZZZZZZZ tmp_c =%d\n",tmp_c);
					break;
				case (int)'V': 
					 key_data = 'v';
					 print_info("[GSL_GESTURE]vvvvvvv tmp_c =%d\n",tmp_c);
					break;
				case (int)'S':
					key_data = 's';
					print_info("[GSL_GESTURE]SSSSSS tmp_c =%d\n",tmp_c);
					break;
				case (int)'*': /*double click*/
					key_data = 0xcc;
					print_info("[GSL_GESTURE]double clickdouble click tmp_c =%d\n",tmp_c);
					break;
				case (int)0xa1fa: /* right */
					key_data = 0xAA;
					print_info("[GSL_GESTURE]rightrightright tmp_c =%d\n",tmp_c);
					break;
				case (int)0xa1fd: /* down */
					key_data = 0xAB;
					print_info("[GSL_GESTURE]downdowndown tmp_c =%d\n",tmp_c);
					break;
				case (int)0xa1fc: /* up */
					key_data = 0xBA;
					print_info("[GSL_GESTURE]upupupup tmp_c =%d\n",tmp_c);
					break;
				case (int)0xa1fb: /* left */			
					key_data = 0xBB;
					print_info("[GSL_GESTURE]leftleftleft tmp_c =%d\n",tmp_c);
					//gsl_gesture_c = (char)tmp_c; 
					break; 
		 
				default: 
					break; 
				} 
				//gesture_key = (char)key_data;
		
			//print_info("gesture_key = %x,gesture_num=%d\n",gesture_key,gesture_num);
			//for(i = 0; i< gesture_num; i++) {
				//printk("gesture_cmd[%d] = %x\n",i ,gesture_cmd[i]);
				//	if(gsl_lcd_flag == 0){ 
					if(key_data !=0){ 
						//gesture_data = gesture_key;
						print_info("[GSL_GESTURE] input report KEY_POWER\n"); 
						input_report_key(gsl_power_idev, KEY_POWER, 1);
						input_sync(gsl_power_idev);
						input_report_key(gsl_power_idev, KEY_POWER, 0);
						input_sync(gsl_power_idev);
						msleep(20);
					print_info("[GSL_GESTURE] set tpd_halt = 0\n"); 
				 	tpd_halt = 0;			
					//break;
				} 
			//}
			goto i2c_lock; 
		} 
#endif
#endif 
	for(i = 1 ;i <= MAX_CONTACTS; i ++)
	{
		if(point_num == 0)
			id_sign[i] = 0;	
		id_state_flag[i] = 0;
	}
	for(i = 0; i < (point_num < MAX_FINGERS ? point_num : MAX_FINGERS); i ++)
	{
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

//tlc open      
#if defined(GSL1680_EXCHANGE_X_Y)
		drv_tempxy = x;
		x = y;
		y = drv_tempxy;
#endif
		print_info("ainenctp [tp-gsl] x  =%d y  =%d \n",x,y);
		if(1 <= id && id <= MAX_CONTACTS)
		{
		#ifdef FILTER_POINT
			filter_point(x, y ,id);
		#else
			record_point(x, y , id);
		#endif
			print_info("ainenctp [tp-gsl] x_new  =%d y_new  =%d \n",x_new,y_new);
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
		tpd_up();
	}
	input_sync(tpd->dev);
i2c_lock:
	print_info("----------------i2c_lock:-----------------\n");	
#ifdef GSL_MONITOR
	i2c_lock_flag = 0;
#endif
}

#ifdef GSL_MONITOR
static void gsl_monitor_worker(void)
{
	char write_buf[4] = {0};
	char read_buf[4]  = {0};
	
	print_info("----------------gsl_monitor_worker-----------------\n");	

#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
        goto queue_monitor_work;
#endif

#ifdef GSL_GESTURE
	if(get_ges_feature_onoff()&&(1== tpd_halt))
		goto queue_monitor_work;
#endif
	
	if(i2c_lock_flag != 0)
		goto queue_monitor_work;
	else
		i2c_lock_flag = 1;
	i2c_smbus_read_i2c_block_data(i2c_client, 0xb0, 4, read_buf);
	if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter ++;
	else
		b0_counter = 0;

	if(b0_counter > 1)
	{
		printk("======read 0xb0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip(i2c_client);
		b0_counter = 0;
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
		init_chip(i2c_client);
	}

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
		init_chip(i2c_client);
		dac_counter = 0;
	}
	i2c_lock_flag = 0;
	
queue_monitor_work:	
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 100);
}
#endif
static int touch_event_handler1(void *unused)
{
			printk("===tong====BOOT_Animation =%d=====\n",BOOT_Animation);

}
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
	
	do
	{
//		enable_irq(touch_irq);//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		printk("===touch_event_handler, task running===\n");

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

#ifdef GSL_GESTURE
#if 0//guodm
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
#endif
static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
	strcpy(info->type, GSLX680_DEV_NAME);
	return 0;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	TPD_DMESG("Device Tree Tpd_irq_registration!");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		ret =
			    request_irq(touch_irq, (irq_handler_t) tpd_eint_interrupt_handler, IRQF_TRIGGER_RISING,
					"TOUCH_PANEL-eint", NULL);
			if (ret > 0) {
				ret = -1;
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}

	} else {
		TPD_DMESG("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	TPD_DMESG("[%s]irq:%d, debounce:%d-%d:", __func__, touch_irq, ints[0], ints[1]);
	return ret;
}

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int err = 0;
	char buffer[5];
	int status=0;
#ifdef TPD_PS_SUPPORT
		struct hwmsen_object obj_ps;
#endif	

//power on, need confirm with SA
	#ifdef TPD_POWER_SOURCE_CUSTOM
		#ifdef CONFIG_ARCH_MT6580
				#if defined(CONFIG_MTK_LEGACY)
				#ifdef TPD_POWER_SOURCE_CUSTOM
				regulator_set_voltage(tpd->reg, 2800000, 2800000);  // set 2.8v
				regulator_enable(tpd->reg);  //enable regulator
				#endif
				#else //guodm 6580 m
				regulator_enable(tpd->reg);
				#endif
		#else
		hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
		#endif
	#else
		hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	#endif
	#ifdef TPD_POWER_SOURCE_1800
		hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
	#endif 


	#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	#ifdef CONFIG_ARCH_MT6580
			//#if defined(CONFIG_MTK_LEGACY)
			#ifdef TPD_POWER_SOURCE_CUSTOM
			regulator_disable(tpd->reg);
			
			regulator_set_voltage(tpd->reg, 2800000, 2800000);  // set 2.8v
			regulator_enable(tpd->reg);  //enable regulator
			#endif
			//#endif
	#else
		hwPowerDown(TPD_POWER_SOURCE,"TP");
		hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
		msleep(100);
	#endif
	#else
		
		msleep(10);
		TPD_DMESG(" fts reset\n");
	    	printk(" fts reset\n");
		#if 1//guodm
		tpd_gpio_output(GTP_RST_PORT,1);
		#else
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	    	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	    	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		#endif
	#endif	
	msleep(100);
	printk("(whs debug)->[tpd_i2c_probe]:gslX680 -->>.\n");

#if 1//guodm custom tp i2c_addr
	client->addr = (0x40);
#endif

    //tpd_set_gpio_output(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
  tpd_gpio_output(GTP_RST_PORT,0);
	msleep(100);
	tpd_gpio_output(GTP_RST_PORT,1);
  //tpd_set_gpio_output(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);

	#if 1//zhub
	tpd_gpio_as_int(GTP_INT_PORT);
	printk(" gslx680 eint\n");
	#else
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);	
	#endif
	msleep(50);
	
	i2c_client = client;	
	

	        if((tpd_read_chip_id(i2c_client) != 0x82) && (tpd_read_chip_id(i2c_client) != 0x83))  //0x82
        {
           return -1;
        }
	init_chip(i2c_client);
	check_mem_data(i2c_client);

	#if 1//ndef MT6589   //zhub
	tpd_irq_registration();
	#else	
	//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING/*EINTF_TRIGGER_FALLING zmlin mod*/, tpd_eint_interrupt_handler, 1);
	//enable_irq(touch_irq);//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	#endif
	
	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}
		thread = kthread_run(touch_event_handler1, 0, "mythread");
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}

#ifdef GSL_GESTURE
    gsl_FunIICRead(gsl_read_oneframe_data);
    gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);
#endif

#ifdef GSL_MONITOR
	printk( "tpd_i2c_probe () : queue gsl_monitor_workqueue\n");

	INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
	gsl_monitor_workqueue = create_singlethread_workqueue("gsl_monitor_workqueue");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1000);
#endif


#ifdef TPD_PROC_DEBUG
#if 0
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
#else
	proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
	gsl_proc_flag = 0;
#endif
#endif

	printk("==tpd_i2c_probe end==\n");

#ifdef TPD_PS_SUPPORT
	//guodm close this,for multi psensor // hwmsen_detach(ID_PROXIMITY);
	obj_ps.self = NULL;
//	obj_ps.self = ogs1688;;
	//	obj_ps.self = cm3623_obj;
	obj_ps.polling = 0;//interrupt mode
	//obj_ps.polling = 1;//need to confirm what mode is!!!
	obj_ps.sensor_operate = tpd_ps_operate;  //gsl1680p_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		printk("attach fail = %d\n", err);
		tpd_proximity_attached = 0;
	}
	printk("y____1 %d\n",err);
	gsl_gain_psensor_data(i2c_client);
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
#endif
#ifdef GSL_GESTURE
#if 0//guodm
    gsl_gesture_init();
#endif
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
//static unsigned short force[] = {0, (GSLX680_ADDR << 1), I2C_CLIENT_END,I2C_CLIENT_END};
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
#endif

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,gslx680"},
	{},
};

struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = GSLX680_DEV_NAME,
	//#ifndef ADD_I2C_DEVICE_ANDROID_4_0	 
		//.owner = THIS_MODULE,
	//#endif
	.of_match_table = tpd_of_match,
	},
			
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = tpd_i2c_id,
	.detect = tpd_i2c_detect,
	#ifndef ADD_I2C_DEVICE_ANDROID_4_0
	//.address_data = &addr_data,
	#endif
};

int tpd_local_init(void)
{
	printk("==tpd_local_init==\n");


#ifdef CONFIG_OF
#ifdef CONFIG_ARCH_MT6580
/*#if defined(CONFIG_MTK_LEGACY)
	if(!tpd->reg){
		tpd->reg=regulator_get(tpd->tpd_dev,PMIC_APP_CAP_TOUCH_VDD); // get pointer to regulator structure
		if (IS_ERR(tpd->reg)) {
			printk("gslx680 regulator_get() failed!\n");
		}else{
			printk("gslx680 regulator_get() OK!\n");
		}
	}*/
#if !defined CONFIG_MTK_LEGACY
	int ret;
	//#error "only support MTK_LEGACY init"
	TPD_DMESG("Device Tree get regulator!");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	/*set 2.8v*/
	if (ret) {
		TPD_DMESG("regulator_set_voltage(%d) failed!\n", ret);
		return -1;
	}
#else
#ifdef TPD_POWER_SOURCE_CUSTOM
//#ifdef CONFIG_OF_TOUCH
#ifdef CONFIG_ARCH_MT6580
		if(!tpd->reg){
			tpd->reg=regulator_get(tpd->tpd_dev,TPD_POWER_SOURCE_CUSTOM); // get pointer to regulator structure
			if (IS_ERR(tpd->reg)) {
				TPD_DMESG("fts regulator_get() failed!\n");
			}else{
			TPD_DMESG("fts regulator_get() OK!\n");
			}
		}
//#endif
//#endif
#endif
#endif
#endif
#endif
#endif


	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	/*
	if(tpd_load_status == 0)
	{
		TPD_DMESG("add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	*/
#ifdef GSL_GESTURE
#if 1//guodm
	set_tpd_gesture_key_arr();
#endif
#endif	
#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
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
	tpd_type_cap = 1;

	printk("==tpd_local_init end==\n");
	return 0;
}

/* Function to manage low power suspend */
static void tpd_suspend(struct early_suspend *h)
{

#ifdef GSL_MONITOR
	printk( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
	cancel_delayed_work_sync(&gsl_monitor_work);
#endif
printk( "==============gsl_ts_suspend () : k================================\n");
#ifdef TPD_PS_SUPPORT
		if(tpd_proximity_flag == 1){
			tpd_proximity_working_in_sleep = 1;
			return 0;}
		tpd_proximity_working_in_sleep = 0;
#endif

	tpd_halt = 1;

#ifdef GSL_GESTURE 
			if(1){ 
				printk( "=============suspend=GSL_GESTURE  satrt : k\n================================\n");
				u8 buf[4]; 
		
				unsigned int temp; 
				//gsl_lcd_flag = 0; 
				msleep(20); 
				buf[0] = 0xa; 
				buf[1] = 0; 
				buf[2] = 0; 
				buf[3] = 0; 
				i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf); 
				printk("--%s--\n",__func__);
				buf[0] = 0; 
				buf[1] = 0; 
				buf[2] = 0x1; 
				buf[3] = 0x5a; 
				i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf); 
				msleep(20); 
				printk( "=============suspend GSL_GESTURE  end : k\n================================\n");
		} 
		else
		{
#endif 

	disable_irq(touch_irq);//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	msleep(20);

	//tpd_set_gpio_output(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	tpd_gpio_output(GTP_RST_PORT,0);
        msleep(20);
      //  hwPowerDown(MT6323_POWER_LDO_VGP2,"TP");
		
#ifdef GSL_GESTURE
		}
#endif
}

/* Function to manage power-on resume */
static void tpd_resume(struct early_suspend *h)
{

/*#ifdef TPD_PS_SUPPORT
	 if(tpd_proximity_flag == 1)
		 return 0;
#endif*/
//add for debug
regulator_set_voltage(tpd->reg, 2800000, 2800000);  // set 2.8v
			regulator_enable(tpd->reg);  //enable regulator

	printk("==tpd_resume==\n");
#ifdef TPD_PS_SUPPORT
	printk("[tp-gsl] [%s] tpd_proximity_flag = %d \n",__func__,tpd_proximity_flag);
    if(tpd_proximity_working_in_sleep==1)//guodm (tpd_proximity_flag == 1)
    {
        //tpd_enable_ps(1);
        tpd_proximity_working_in_sleep = 0;
        goto enable_ps;
    }
#endif
//end	
	msleep(20);
printk( "=============tpd_resumen================================\n");
#ifdef GSL_GESTURE 
	if(1){ 
		printk( "=============tpd_resumen= gsture start===============================\n");
		u8 buf[4]; 
		unsigned int temp; 
		//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO); 
		tpd_gpio_output(GTP_RST_PORT,0);
		msleep(20); 
		//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE); 
		tpd_gpio_output(GTP_RST_PORT,1);
		
		msleep(20); 
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
		printk( "=============tpd_resumen= gsture end===============================\n");

	}
	else
	{
#endif 

	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
  //hwPowerOn(MT6323_POWER_LDO_VGP2,VOL_2800,"TP");
	msleep(20);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	tpd_gpio_output(GTP_RST_PORT,1);
	msleep(50);
	enable_irq(touch_irq);//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef GSL_GESTURE
	}
#endif
	
#ifdef TPD_PS_SUPPORT	
enable_ps:	
	;
#endif
	printk( "gsl tpd_resume () 3333333333333333 GSL_GESTUREGSL\n");
	reset_chip(i2c_client);
	startup_chip(i2c_client);
	check_mem_data(i2c_client);	


#ifdef TPD_PS_SUPPORT
	 if(tpd_proximity_flag == 1)
	{
		tpd_enable_ps(1);
		return;
    	}
#endif



#ifdef GSL_MONITOR
		printk( "gsl_ts_resume () : queue gsl_monitor_work\n");
		queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
#endif	

	tpd_halt = 0;
	//add for debug
//end
}

//guodm add for proc info
static int tp_info_read(struct seq_file *m, void *v){
	int ret = 0;

	ret = seq_printf(m, "Sileadinc gslX680\n");

//#ifdef GSL_IDENTY_TP	
//	ret = seq_printf(m, "identy:[%d],0x%x,0x%x,0x%x,0x%x\n",gsl_identyarr[4],gsl_identyarr[0],gsl_identyarr[1],gsl_identyarr[2],gsl_identyarr[3]);
//#endif

#ifdef TPD_PS_SUPPORT
	if(tpd_proximity_attached == 1)
		ret = seq_printf(m, "proximity:[attached | %s]\n",(tpd_proximity_flag==1)?"on":"off");
	else
		ret = seq_printf(m, "proximity:[unattached]\n");
#else
	ret = seq_printf(m, "proximity:[none]\n");
#endif

#ifdef GSL_GESTURE
	ret = seq_printf(m, "gesture:[attached | %s]\n",get_ges_feature_onoff()?"on":"off");
#else
	ret = seq_printf(m, "gesture:[none]\n");
#endif

	return ret;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = GSLX680_DEV_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#if defined(TPD_HAVE_BUTTON)||defined(GSL_GESTURE) //gesture use button input dev
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
	.tpd_info_read = tp_info_read,
};

/* called when loaded into kernel */
static int __init gsl_tpd_driver_init(void) {
	printk("Sileadinc gslX680 touch panel driver init\n");
	
#ifdef ADD_I2C_DEVICE_ANDROID_4_0
	i2c_register_board_info(I2C_CAP_TOUCH_CHANNEL, &gslX680_i2c_tpd, 1);	
#endif
	tpd_get_dts_info();
	
	if(tpd_driver_add(&tpd_device_driver) < 0)
		printk("add gslX680 driver failed\n");
	return 0;
}

/* should never be called */
static void __exit gsl_tpd_driver_exit(void) {
	printk("Sileadinc gslX680 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(gsl_tpd_driver_init);
module_exit(gsl_tpd_driver_exit);


