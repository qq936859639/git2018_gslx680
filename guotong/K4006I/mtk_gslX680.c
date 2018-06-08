
#include <linux/init.h>
#include <linux/workqueue.h>
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
//#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
//#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
//#include <linux/rtpm_prio.h>
#include "tpd.h"
#include <upmu_common.h>
#include <mt_boot.h>
#include "mtk_gslX680.h"



//#if TPD_SUPPORT_I2C_DMA
//#include <linux/dma-mapping.h>
//#endif

#ifdef CONFIG_GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>


//#define TP_GESTURE_PROC_FILE   "tp_gesture"




#define TPD_PROXIMITY

#ifdef TPD_PROXIMITY
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <linux/wakelock.h>    //added by xen 20140904
static u8 tpd_proximity_flag = 0; //flag whether start alps
static u8 tpd_proximity_detect = 20;//0-->close ; 20--> far away
static struct wake_lock ps_lock;
static u8 gsl_psensor_data[8]={0};
#endif

// #define GTP_RST_PORT    0
// #define GTP_INT_PORT    1
unsigned int gsl_touch_irq = 0;
static struct mutex gsl_i2c_lock;

//static struct mutex gsl_i2c_lock;
 //#define GSL_DEBUG
// wanghe 2014-03-18 open for esd check. maybe should change .h 0x74 & 0x7c
#define GSLX680_NAME	"gslX680"
#define GSLX680_ADDR	0x40
#define MAX_FINGERS	  	10
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
#define SMBUS_TRANS_LEN	0x01 //update by liuxiaoliang 0x08  wanghe must use this!!
#define GSL_PAGE_REG		0xf0
//#define ADD_I2C_DEVICE_ANDROID_4_0
#define HIGH_SPEED_I2C
//#define FILTER_POINT   // delete by wanghe 2013-08-27  xiu gai xian xing du for wm001 baicheng
#ifdef FILTER_POINT
#define FILTER_MAX	6
#endif

#define TPD_PROC_DEBUG

#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>  //lzk
//static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

static int gsl_up_flag = 0;
#ifdef GSL_IDENTY_TP
static int gsl_tp_type = 0;
//static u32 array_len=0;
static unsigned int *gsl_config_data_id= gsl_config_data_id_two; //= gsl_config_data_id_968;
static int gsl_identify_tp(struct i2c_client *client);
#endif
static int tpd_flag = 0;
static int tpd_halt=0;
static char eint_flag = 0;
extern struct tpd_device *tpd;
static const struct of_device_id gslx68x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

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
//static char dac_counter = 0;
static char b0_counter = 0;
static char bc_counter = 0;
static char i2c_lock_flag = 0;
#endif


//#define GSL_GESTURE   


#ifdef GSL_GESTURE
static int gsl_gesture_flag = 1;
static struct input_dev *gsl_power_idev;
static int gsl_lcd_flag = 0;
static char gsl_gesture_c = 0;
static struct i2c_client *i2c_client;

static volatile int gsl_halt_flag = 0;


extern  int  gsl_obtain_gesture(void);
extern void kpd_touchpanel_gesture_handler(int key_code);
extern void gsl_GestureExternInt(unsigned int *model,int len); 

extern void gsl_FunIICRead(unsigned int (*ReadIICInt)(unsigned int *, unsigned int, unsigned int));
#endif

static u32 id_sign[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;


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
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
		kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
		kal_bool auto_umask);
#endif

#define GSL_TPD_DEBUG   
#ifdef GSL_TPD_DEBUG
#define GSL_DUBGE(fmt, args...) do{ \
					printk("gsl->debug:" fmt, ##args);\
				}while(0)
#define FUNC_ENTRY()  printk("gsl->debug:%s, entry\n", __func__)
#define FUNC_EXIT()   printk("gsl->debug:%s, exit\n", __func__)
#else
#define GSL_DUBGE(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
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

//#ifdef GSL_GESTURE //xen 20160406
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
	//mutex_lock(&gsl_i2c_lock);//xen 20160406
	if(temp < 0x80)
	{
		temp = (reg + 8) & 0x5c;
		//i2c_master_send(client,&temp,1);	//xen 20160406
		err = i2c_smbus_read_i2c_block_data(client, temp, 4, &buf[0]);//i2c_master_recv(client,&buf[0],4);
		temp = reg;
		//i2c_master_send(client,&temp,1);  //xen 20160406	
		err = i2c_smbus_read_i2c_block_data(client, temp, 4, &buf[0]);//i2c_master_recv(client,&buf[0],4);
	}
	
	for(i=0;i<num;)
	{	
		temp = reg + i;
		//i2c_master_send(client,&temp,1);  //xen 20160406
		if((i+8)<num)
			err = i2c_smbus_read_i2c_block_data(client, temp, 8, (buf+i));//i2c_master_recv(client,(buf+i),8);
		else
			err = i2c_smbus_read_i2c_block_data(client, temp, (num-i), (buf+i));//i2c_master_recv(client,(buf+i),(num-i));
		i+=8;
	}
	//mutex_unlock(&gsl_i2c_lock);//xen 20160406

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

#if 1//defined(YK610_CUSTOMER_GUANGLIAN_Q31_A_QHD)

	struct i2c_msg xfer_msg[1]= {{0}};//modify crystal
	int err;
	u8 tmp_buf[num + 1];

	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = 0;//client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	xfer_msg[0].timing = 400;//I2C_TRANS_SPEED;
	
	mutex_lock(&gsl_i2c_lock);
	err = i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);
#else
		int err;
		err = i2c_smbus_write_i2c_block_data(client, reg, num, buf);
#endif

	
	return err;
}
static void startup_chip(struct i2c_client *client)
{
	//char write_buf = 0x00;
	    u8 buf[4] = {0};
        #if 1//defined(YK610_CUSTOMER_GUANGLIAN_Q31_A_QHD)// add by fully 2016-06-17
		buf[3] = 0x01;
		buf[2] = 0xfe;
		buf[1] = 0x0e;
		buf[0] = 0x02;
		gsl_write_interface(client,0xf0,buf,4);
		buf[3] = 0x0;
		buf[2] = 0xa;
		buf[1] = 0x0;
		buf[0] = 0x70;
		gsl_write_interface(client,0x4,buf,4);
		
		buf[0]=0x01;
		gsl_write_interface(client,0x88,buf,4);
		buf[0]=0;
        #endif	
	
		gsl_write_interface(client,0xe0,buf,4);
	  //i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf); 	
#ifdef GSL_NOID_VERSION
       #ifdef GSL_IDENTY_TP
        if(1==gsl_tp_type)
		gsl_DataInit(gsl_config_data_id_one);
          if(2==gsl_tp_type)
		gsl_DataInit(gsl_config_data_id_two);	
		  if(3==gsl_tp_type)
		gsl_DataInit(gsl_config_data_id_three);
        #else
	    gsl_DataInit(gsl_config_data_id);
         #endif
#endif
	msleep(10);		
}

static void reset_chip(struct i2c_client *client)
{
	char write_buf[4]	= {0};

	write_buf[0] = 0x88;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(10);

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
	msleep(10);

	write_buf[0] = 0x03;  // 0x01;  wanghe change to 0x03 2014-03-18
	i2c_smbus_write_i2c_block_data(client, 0x80, 1, &write_buf[0]); 	
	msleep(5);
	
	write_buf[0] = 0x04;
	i2c_smbus_write_i2c_block_data(client, 0xe4, 1, &write_buf[0]); 	
	msleep(5);

	write_buf[0] = 0x00;
	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]); 	
	msleep(10);
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

#ifdef GSL_IDENTY_TP

static void gsl_load_fw(struct i2c_client *client,struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[SMBUS_TRANS_LEN*4+4] = {0};
	u8 reg = 0, send_flag = 1, cur = 0;
	
	u32 source_line = 0;
	u32 source_len = 0;
	struct fw_data *ptr_fw;
		{
			ptr_fw = GSL_DOWNLOAD_DATA;
			source_len = data_len;
		}
	client->timing = 400;

	FUNC_ENTRY();

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
				buf[0] = (u8)(ptr_fw[source_line].val & 0x000000ff);
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

	FUNC_EXIT();

}


#else
static void gsl_load_fw(struct i2c_client *client)

{
    u8 buf[SMBUS_TRANS_LEN*4] = {0};
    u8 reg = 0, send_flag = 1, cur = 0;

    unsigned int source_line = 0;
	
    unsigned int source_len = ARRAY_SIZE(GSLX680_FW);

	client->timing = 400;

	FUNC_ENTRY();

	for (source_line = 0; source_line < source_len; source_line++) 
	{
	    
		if(1 == SMBUS_TRANS_LEN)
		{
            reg = GSLX680_FW[source_line].offset;
            memcpy(&buf[0], &GSLX680_FW[source_line].val, 4);
            i2c_smbus_write_i2c_block_data(client, reg, 4, buf); 	
        }
        else
        {
            /* init page trans, set the page val */
            if (GSL_PAGE_REG == GSLX680_FW[source_line].offset)
            {
                buf[0] = (u8)(GSLX680_FW[source_line].val & 0x000000ff);
                i2c_smbus_write_i2c_block_data(client, GSL_PAGE_REG, 1, &buf[0]); 	
                send_flag = 1;
            }
            else 
            {
                if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
                    reg = GSLX680_FW[source_line].offset;

                memcpy(&buf[cur], &GSLX680_FW[source_line].val, 4);
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

	FUNC_EXIT();

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
		GSL_DUBGE("<<<< wanghe gslX680 I read reg 0xf0 is %x\n", read_buf);

	msleep(2);
	ret = i2c_smbus_write_i2c_block_data( client, 0xf0, 1, &write_buf );
	if(ret  >=  0 )
		GSL_DUBGE("<<<< wanghe gslX680 I write reg 0xf0 0x12\n");
	
	msleep(2);
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if(ret <  0 )
		rc --;
	else
		GSL_DUBGE("<<<< wanghe gslX680 I read reg 0xf0 is 0x%x\n", read_buf);

	return rc;
}
#ifdef GSL_IDENTY_TP
static void init_chip(struct i2c_client *client)
{
	int rc;	
	u32 tmp;

	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	//msleep(20); 	
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	//msleep(20); 		
	
	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(10);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(10);

	rc = test_i2c(client);
	if(rc < 0)
	{
		printk("------gslX680 test_i2c error------\n");	
		return;
	}else
		printk("------gslX680 test_i2c success------\n");	
	
	clr_reg(client);
	reset_chip(client);
	if(0==gsl_tp_type)
		gsl_identify_tp(client);		
	reset_chip(client);
	clr_reg(client);
	reset_chip(client);

	if(1==gsl_tp_type){
		tmp = ARRAY_SIZE(GSLX680_FW_ONE);
		gsl_load_fw(client,GSLX680_FW_ONE,tmp);
	}
	else if(2==gsl_tp_type){
		tmp = ARRAY_SIZE(GSLX680_FW_TWO);
		gsl_load_fw(client,GSLX680_FW_TWO,tmp);
	}
	else if(3==gsl_tp_type){
		tmp = ARRAY_SIZE(GSLX680_FW_THREE);
		gsl_load_fw(client,GSLX680_FW_THREE,tmp);
	}
	
	startup_chip(client);
	reset_chip(client);
	startup_chip(client);		
}
#else
static void init_chip(struct i2c_client *client)
{
	int rc;
	
	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	//msleep(20); 	
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	//msleep(20); 		
	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(10);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(10);
	GSL_DUBGE("------gsl1111111111 test_i2c error------\n");

	rc = test_i2c(client);
	if(rc < 0)
	{
		printk("------gslX680 test_i2c error------\n");	
		return;
	}else
		printk("------gslX680 test_i2c success------\n");	

	clr_reg(client);
	reset_chip(client);
	gsl_load_fw(client);			
	startup_chip(client);
	reset_chip(client);
	startup_chip(client);		
}
#endif

static void check_mem_data(struct i2c_client *client)
{
	char read_buf[4]  = {0};
	
	msleep(10);
	i2c_smbus_read_i2c_block_data(client,0xb0, sizeof(read_buf), read_buf);
	GSL_DUBGE("------gsl1111111111 check_mem_data error------\n");
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		GSL_DUBGE("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
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
		GSL_DUBGE("[GSL_GESTURE] gsl_sysfs_tpgesturet_store off.\n");
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
		GSL_DUBGE("[GSL_GESTURE] gsl_sysfs_tpgesturet_store on.\n");
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
/*	if(rc){
		input_free_device(idev);
		gsl_power_idev = NULL;
	}
*/
}
static unsigned int gsl_gesture_init(void)
{
	int ret;
	struct kobject *gsl_debug_kobj;
	gsl_debug_kobj = kobject_create_and_add("gsl_gesture", NULL) ;
	if (gsl_debug_kobj == NULL)
	{
		GSL_DUBGE("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_tpgesture.attr);
    if (ret)
    {
        GSL_DUBGE("%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }

	gsl_request_power_idev();
	


	GSL_DUBGE("[GSL_GESTURE] gsl_gesture_init success.\n");
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
//static int gt91xx_config_read_proc(struct file *file, char *buffer, size_t count, loff_t *ppos)
//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	//char *ptr = NULL;//page;
	char temp_data[5] = {0};
	unsigned int tmp=0;
	//unsigned int *ptr_fw;
	
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_NOID_VERSION
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		//ptr += sprintf(ptr,"version:%x\n",tmp);
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_NOID_VERSION 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<512/*ARRAY_SIZE(gsl_config_data_id)*/) //xjl 20160524
			{
					//ptr +=sprintf(ptr,"%d\n",gsl_config_data_id[tmp]); 
					seq_printf(m,"%d\n",gsl_config_data_id[tmp]);
			}
#endif
		}
		else 
		{
			i2c_smbus_write_i2c_block_data(i2c_client,0Xf0,4,&gsl_data_proc[4]);
			if(gsl_data_proc[0] < 0x80)
				i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);

			//ptr +=sprintf(ptr,"offset : {0x%02x,0x",gsl_data_proc[0]);
			//ptr +=sprintf(ptr,"%02x",temp_data[3]);
			//ptr +=sprintf(ptr,"%02x",temp_data[2]);
			//ptr +=sprintf(ptr,"%02x",temp_data[1]);
			//ptr +=sprintf(ptr,"%02x};\n",temp_data[0]);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
	//*eof = 1;
	//return (ptr - page);
	return 0;
}
//static int gsl_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
ssize_t  gsl_config_write_proc(struct file *file, const char *buffer, size_t  count, loff_t  *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	
	GSL_DUBGE("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		GSL_DUBGE("size not match [%d:%d]\n", CONFIG_LEN, count);
        return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		GSL_DUBGE("alloc path_buf memory error \n");
	}	
	//if(copy_from_user(path_buf, buffer, (count<CONFIG_LEN?count:CONFIG_LEN)))
	if(copy_from_user(path_buf, buffer, count))
	{
		GSL_DUBGE("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	GSL_DUBGE("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	
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
		GSL_DUBGE("gsl version\n");
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
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<512/*ARRAY_SIZE(gsl_config_data_id)*/)
		{
			gsl_config_data_id[tmp1] = tmp;
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}


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

#ifdef TPD_PROXIMITY
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
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] = 0x5a;
		buf[2] = 0x5a;
		buf[1] = 0x5a;
		buf[0] = 0x5a;
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);

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
		GSL_DUBGE("tpd-ps function is on\n");
	}
	else 
	{
		tpd_proximity_flag = 0;
		wake_unlock(&ps_lock);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x3;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] = gsl_psensor_data[3];
		buf[2] = gsl_psensor_data[2];
		buf[1] = gsl_psensor_data[1];
		buf[0] = gsl_psensor_data[0];

		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);

		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		i2c_smbus_write_i2c_block_data(i2c_client, 0xf0, 4, buf);
		buf[3] = gsl_psensor_data[7];
		buf[2] = gsl_psensor_data[6];
		buf[1] = gsl_psensor_data[5];
		buf[0] = gsl_psensor_data[4];
		i2c_smbus_write_i2c_block_data(i2c_client, 0x00, 4, buf);	
		GSL_DUBGE("tpd-ps function is off\n");
	}
	return 0;
}

int tpd_ps_operate_gslX680(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *sensor_data;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSL_DUBGE("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSL_DUBGE("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if((tpd_enable_ps(1) != 0))
					{
						GSL_DUBGE("enable ps fail: %d\n", err);
						return -1;
					}
				//					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						GSL_DUBGE("disable ps fail: %d\n", err);
						return -1;
					}
				//					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSL_DUBGE("get sensor data parameter error!\n");
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
			GSL_DUBGE("proxmy sensor operate function no this parameter %d!\n", command);
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

	GSL_DUBGE("[pre] 0xb4 = %x \n",gsl_timer_data);
	GSL_DUBGE("[cur] 0xb4 = %x \n",tmp);
	GSL_DUBGE("gsl_timer_flag=%d\n",gsl_timer_flag);
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

#ifdef GSL_IDENTY_TP	
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
	int err=1;
	int flag=0;
	unsigned int tmp,tmp0;
	unsigned int tmp1,tmp2,tmp3; //,tmp4;
	u32 num;

identify_tp_repeat:
	clr_reg(client);
	reset_chip(client);
	num = ARRAY_SIZE(GSL_TP_CHECK_FW);
	gsl_load_fw(client,GSL_TP_CHECK_FW,num);
	startup_chip(client);
	msleep(200);
	i2c_smbus_read_i2c_block_data(client,0xb4,4,buf);
	GSL_DUBGE("the test 0xb4 = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],buf[1],buf[0]);

	if (((buf[3] << 8) | buf[2]) > 1) {
		GSL_DUBGE("[TP-GSL][%s] is start ok\n",__func__);
		msleep(20);
		i2c_smbus_read_i2c_block_data(client,0xb8,4,buf);
		tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		GSL_DUBGE("the test 0xb8 = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],buf[1],buf[0]);

		tmp1 = gsl_count_one(GSL_CHIP_1^tmp);
		tmp0 = gsl_count_one((tmp&GSL_CHIP_1)^GSL_CHIP_1); 
		tmp1 += tmp0*GSL_C;
		GSL_DUBGE("[TP-GSL] tmp1 = %d\n",tmp1);
		
		tmp2 = gsl_count_one(GSL_CHIP_2^tmp); 
		tmp0 = gsl_count_one((tmp&GSL_CHIP_2)^GSL_CHIP_2);
		tmp2 += tmp0*GSL_C;
		GSL_DUBGE("[TP-GSL] tmp2 = %d\n",tmp2);

		tmp3 = gsl_count_one(GSL_CHIP_3^tmp); 
		tmp0 = gsl_count_one((tmp&GSL_CHIP_3)^GSL_CHIP_3);
		tmp3 += tmp0*GSL_C;
		GSL_DUBGE("[TP-GSL] tmp3 = %d\n",tmp3);
		
		if (0xffffffff == GSL_CHIP_1) {
			tmp1=0xffff;
		}
		if (0xffffffff == GSL_CHIP_2) {
			tmp2=0xffff;
		}
		if (0xffffffff == GSL_CHIP_3) {
			tmp3=0xffff;
		}
		GSL_DUBGE("[TP-GSL] tmp1 = %d\n",tmp1);
		GSL_DUBGE("[TP-GSL] tmp2 = %d\n",tmp2);
		GSL_DUBGE("[TP-GSL] tmp3 = %d\n",tmp3);
		
		tmp = tmp1;
		if (tmp1 > tmp2) {
			tmp = tmp2; 
		}
		if (tmp > tmp3) {
			tmp = tmp3; 
		}
		if(tmp == tmp1){
			gsl_config_data_id = gsl_config_data_id_one;
			gsl_tp_type = 1;
		} else if(tmp == tmp2) {
			gsl_config_data_id = gsl_config_data_id_two;
			gsl_tp_type = 2;
		}
		 else if(tmp == tmp3) {
			gsl_config_data_id = gsl_config_data_id_three;
			gsl_tp_type = 3;
		}
		err = 1;
	} else {
		flag++;
		if(flag < 3) {
			goto identify_tp_repeat;
		}
		err = 0;
	}
	return err; 
}
#endif

void tpd_down( int id, int x, int y, int p) 
{
	GSL_DUBGE("------tpd_down id: %d, x:%d, y:%d------ \n", id, x, y);

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
		 GSL_DUBGE("D virtual key \n");
	}
	#endif	
}

void tpd_up(void) 
{
	GSL_DUBGE("------tpd_up------ \n");

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
	u8 touch_data[MAX_FINGERS * 4 + 4] = {0};
	u8 buf[4] = {0};
	unsigned char point_num = 0;
	unsigned int x, y, temp_a, temp_b, i,tmp1,id;
#ifdef GSL_NOID_VERSION
	struct gsl_touch_info cinfo={{0},{0},{0},0};
#endif
		
#ifdef TPD_PROXIMITY
		int err;
		struct hwm_sensor_data sensor_data;
		if (tpd_proximity_flag == 1)
		{
			i2c_smbus_read_i2c_block_data(i2c_client,0xac,4,buf);
			GSL_DUBGE("gslX680   buf[0] = %d buf[1] = %d,  buf[2] = %d  buf[3] = %d \n",buf[0],buf[1],buf[2],buf[3]);
			
			if (buf[0] == 1 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
			{
				tpd_proximity_detect = 0;
			}
			else
			{
				tpd_proximity_detect = 20;
			}
			GSL_DUBGE("gslX680    ps change   tpd_proximity_detect = %d  \n",tpd_proximity_detect);
			//map and store data to hwm_sensor_data
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			//let up layer to know
			if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			{
				GSL_DUBGE("call hwmsen_get_interrupt_data fail = %d\n", err);
			}
		}
#endif
	//GSL_DUBGE("<xieen>===report_data_handle\n");
#ifdef GSL_MONITOR
	//if(2==gsl_timer_flag){
	//	return;
	//}
	if(i2c_lock_flag != 0)
	{
		GSL_DUBGE("<<<< wanghe %s() %d\n", __func__, __LINE__);
		return;
	}
	else
		i2c_lock_flag = 1;
#endif


#if 0//def TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
    {
    	// GSL_DUBGE("<<<< wanghe gsl_proc_flag == 1");
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
	GSL_DUBGE("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(i = 0; i < (point_num < MAX_CONTACTS ? point_num : MAX_CONTACTS); i ++)
	{
		temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
		temp_b = touch_data[(i + 1) * 4 + 2];
		cinfo.x[i] = temp_a << 8 |temp_b;
		temp_a = touch_data[(i + 1) * 4 + 1];
		temp_b = touch_data[(i + 1) * 4 + 0];
		cinfo.y[i] = temp_a << 8 |temp_b;		
		GSL_DUBGE("tp-gsl  x = %d y = %d \n",cinfo.x[i],cinfo.y[i]);
	}
	cinfo.finger_num = (touch_data[3]<<24)|(touch_data[2]<<16)|
		(touch_data[1]<<8)|touch_data[0];
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	GSL_DUBGE("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		GSL_DUBGE("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
	}
	point_num = cinfo.finger_num;
#endif

#ifdef GSL_GESTURE
	//GSL_DUBGE("[GSL_GESTURE] tpd_halt =%d, gsl_gesture_flag = %d \n",gsl_halt_flag, gsl_gesture_flag);
	if((gsl_halt_flag == 1)&&(gsl_gesture_flag == 1)){
		int ges_key = 0;
		int tmp_c;
		tmp_c = gsl_obtain_gesture();
		GSL_DUBGE("[GSL_GESTURE] tmp_c =%d \n",tmp_c);
		
		switch(tmp_c){
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
				gsl_gesture_c = (char)tmp_c;
				if(gsl_lcd_flag == 0){
					/*
					GSL_DUBGE("[GSL_GESTURE] input report KEY_POWER\n");
					input_report_key(gsl_power_idev,KEY_POWER,1);
					input_sync(gsl_power_idev);
					input_report_key(gsl_power_idev,KEY_POWER,0);
					input_sync(gsl_power_idev);
					*/
				}
				//GSL_DUBGE("[GSL_GESTURE] set gsl_lcd_flag = 1\n");
				gsl_lcd_flag = 1;			
				switch(tmp_c){
					case (int)'C':
						ges_key = KEY_TPGESTURE_C;
						break;
					case (int)'E':
						ges_key = KEY_TPGESTURE_E;
						break;
					case (int)'M':
						ges_key = KEY_TPGESTURE_M;
						break;
					case (int)'O':
						ges_key = KEY_TPGESTURE_O;
						break;
					case (int)'S':
						ges_key = KEY_TPGESTURE_S;
						break;
					case (int)'V':
						ges_key = KEY_TPGESTURE_V;
						break;
					case (int)'W':
						ges_key = KEY_TPGESTURE_W;
						break;
					case (int)'Z':
						ges_key = KEY_TPGESTURE_Z;
						break;
					case (int)'*': //double click
						ges_key = KEY_POWER;
						break;
					case 0xa1fa: //right
						ges_key = KEY_TPGESTURE_RIGHT;
						break;
					case 0xa1fb: //left
						ges_key = KEY_TPGESTURE_LEFT;
						break;
					case 0xa1fc: //up
						ges_key = KEY_TPGESTURE_UP;
						break;
					case 0xa1fd: //down
						ges_key = KEY_TPGESTURE_DOWN;
						break;
					default:
						break;
				}
				kpd_touchpanel_gesture_handler(ges_key);
				
				reset_chip(i2c_client);
				startup_chip(i2c_client);
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
		#if defined(YK610_CUSTOMER_EMMC_GELI_G102_QHD_50)
			tpd_down(id, x, y, 10);
		#else
		    tpd_down(id, x_new, y_new, 10);
		#endif
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
			// GSL_DUBGE("<<<< wanghe 1==gsl_up_flag");
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
static void gsl_monitor_worker(struct work_struct *work)
{
	//u8 write_buf[4] = {0};
	u8 read_buf[4]  = {0};
	char init_chip_flag = 0;
	
	FUNC_ENTRY();
	
#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
    {
    	// GSL_DUBGE("<<<< wanghe gsl_proc_flag == 1");
        return;
    }
#endif	

	if(i2c_lock_flag != 0)
		goto queue_monitor_work;
	else
		i2c_lock_flag = 1;

	// GSL_DUBGE("----------------<<<< wanghe gsl_monitor_worker i2c_lock_flag to 1 -----------------\n");	

	i2c_smbus_read_i2c_block_data(i2c_client, 0xb0, 4, read_buf);
	if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter ++;
	else
		b0_counter = 0;

	if(b0_counter > 1)
	{
		//GSL_DUBGE("======read 0xb0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
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
		//GSL_DUBGE("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",int_1st[3], int_1st[2], int_1st[1], int_1st[0], int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
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
		//GSL_DUBGE("<<<< wanghe ======read 0xbc: %x %x %x %x======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
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
		//GSL_DUBGE("======read DAC1_0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		dac_counter = 0;
	}
#endif

	FUNC_EXIT();
queue_monitor_init_chip:
	if(init_chip_flag)
		init_chip(i2c_client);
	
	i2c_lock_flag = 0;
	 //GSL_DUBGE("----------------<<<< wanghe gsl_monitor_worker i2c_lock_flag to 0 -----------------\n");	
queue_monitor_work:	
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 500);//50
	//i2c_lock_flag = 0;
}
#endif

static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);
	
	do
	{
		//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		//enable_irq(gsl_touch_irq);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		disable_irq(gsl_touch_irq);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);

		eint_flag = 0;
		report_data_handle();
		enable_irq(gsl_touch_irq);
	} while (!kthread_should_stop());
	
	msleep(50);
	/* EINT device tree, default EINT enable */
	//tpd_irq_registration();

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{

	eint_flag = 1;
	tpd_flag=1; 
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	FUNC_ENTRY();
	
	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		/*gsl_touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		gsl_touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(gsl_touch_irq, tpd_eint_interrupt_handler,IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if (ret > 0)
				GSL_DUBGE("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		GSL_DUBGE("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	GSL_DUBGE("[%s]xxx gsl------------------------tpd request_irq can not find touch eint device node!.", __func__);
	FUNC_EXIT();
	return 0;
}


//#define GSL_COMPATIBLE    // wanghe 2013-07-30 must add this

#ifdef GSL_COMPATIBLE
static int gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4];
	int i,err=-1;
	
	FUNC_ENTRY();
	
	for(i=0;i<3;i++)
	{
		err = i2c_smbus_read_i2c_block_data(client,0xfc,4,buf);
		GSL_DUBGE("[ gslX680  0xfc = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],
			buf[1],buf[0]);
		if(err>0)
		{
			break;
		}
	}
	return err;	
}
#endif
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) {
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

#ifdef GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i;
	GSL_DUBGE("tp-gsl-gesture %s\n",__func__);

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

static int of_get_gslx68x_platform_data(struct device *dev)
{
	

	FUNC_ENTRY();

	if (dev->of_node) {
		const struct of_device_id *match;
		GSL_DUBGE("==gslX680  go in (dev->of_node)==\n");
		match = of_match_device(of_match_ptr(gslx68x_dt_match), dev);
		if (!match) {
			GSL_DUBGE("gslX680  Error: No device match found\n");
			return -ENODEV;
		}
	}
		GSL_DUBGE("==gslX680  of_get_gslx68x_platform_data  end ==\n");
	return 0;
}

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int err = 0;
	int ret = -1	;
	//char buffer[5];
	//int status=0;
#ifdef TPD_PROXIMITY
	int errpro =0;
		struct hwmsen_object obj_ps;
#endif

	mutex_init(&gsl_i2c_lock);

	FUNC_ENTRY();

	of_get_gslx68x_platform_data(&client->dev);

	tpd_gpio_output(GTP_RST_PORT, 0);
	
	msleep(10);//100
	ret = regulator_enable(tpd->reg);
	if (ret != 0)
		GSL_DUBGE("gslX680  Failed to enable reg-vgp2: %d\n", ret);

	
	msleep(10);//100
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(10);//100

	
	tpd_gpio_as_int(GTP_INT_PORT);
	
	msleep(10);//100


#ifdef GSL_COMPATIBLE
	err = gsl_compatible_id(client);
	if(err<0)
	{
			GSL_DUBGE("==gslX680  gsl_compatible_id==\n");
		return err;
	}
#endif
	i2c_client = client;
		if(i2c_client->addr !=0x40)
		{	
			GSL_DUBGE("==gslX680 -----------erro----------addr=%d\n",i2c_client->addr);
			i2c_client->addr=0x40;
			
		}

		disable_irq(gsl_touch_irq);
		msleep(100);
		GSL_DUBGE("==gslX680  	disable_irq(gsl_touch_irq);==\n");
#ifdef GSL_MONITOR
    i2c_client->timing = 400;
#endif
#ifdef GSL_GESTURE
	gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);

	gsl_FunIICRead(gsl_read_oneframe_data);
#endif
	init_chip(i2c_client);
	check_mem_data(i2c_client);
	tpd_irq_registration();
	  //mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	  //mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	  //mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	  //mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);	
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 0);
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 0);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		
		GSL_DUBGE(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}

#ifdef GSL_MONITOR
	GSL_DUBGE( "tpd_i2c_probe () : queue gsl_monitor_workqueue\n");

	INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
	gsl_monitor_workqueue = create_workqueue("gsl_monitor_workqueue");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1000);//1000
#endif

    // added by xen for gesture flag, 20141008
    //remove_proc_entry(TP_GESTURE_PROC_FILE, NULL);

#ifdef TPD_PROC_DEBUG
#if 0  // xen 20150323
	gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
	//GSL_DUBGE("[tp-gsl] [%s] gsl_config_proc = %x \n",__func__,gsl_config_proc);
	if (gsl_config_proc == NULL)
	{
		GSL_DUBGE("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	else
	{
		gsl_config_proc->read_proc = gsl_config_read_proc;
		gsl_config_proc->write_proc = gsl_config_write_proc;
	}
#else
	proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
#endif
	gsl_proc_flag = 0;
#endif

	
#ifdef TPD_PROXIMITY
		//obj_ps.self = gsl1680p_obj;
		//	obj_ps.self = cm3623_obj;
		hwmsen_detach(ID_PROXIMITY);
		obj_ps.self = NULL;
		obj_ps.polling = 0;//interrupt mode
		//obj_ps.polling = 1;//need to confirm what mode is!!!
		obj_ps.sensor_operate = tpd_ps_operate_gslX680;//gsl1680p_ps_operate;
		if((errpro = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
		{
			
			GSL_DUBGE("attach fail = %d\n", errpro);
		}
		
		GSL_DUBGE("y____1 \n");
		gsl_gain_psensor_data(i2c_client);
		wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
#endif
#ifdef GSL_GESTURE
	gsl_gesture_init();
#endif
	enable_irq(gsl_touch_irq);
	FUNC_EXIT();
	return 0;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	FUNC_ENTRY();
	
	return 0;
}


//static const struct i2c_device_id tpd_i2c_id[] = {{TPD_DEVICE,0},{}};
static const struct i2c_device_id tpd_i2c_id[] = {{"gslX680",0},{}};
// #ifdef ADD_I2C_DEVICE_ANDROID_4_0
// static struct i2c_board_info __initdata gslX680_i2c_tpd={ I2C_BOARD_INFO("gslX680", (GSLX680_ADDR))};
// #else
// static unsigned short force[] = {0, (GSLX680_ADDR << 1), I2C_CLIENT_END,I2C_CLIENT_END};
// static const unsigned short * const forces[] = { force, NULL };
// static struct i2c_client_address_data addr_data = { .forces = forces,};
// #endif

struct i2c_driver tpd_i2c_driver = {
	.driver = {
		// .name = TPD_DEVICE,
	.name = "gslX680",  // changed by wanghe 2013-07-27 for sys/bus/i2c/drivers
	.of_match_table = of_match_ptr(gslx68x_dt_match),

	// #ifndef ADD_I2C_DEVICE_ANDROID_4_0	 
	// 	.owner = THIS_MODULE,
	// #endif
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = tpd_i2c_id,
	.detect = tpd_i2c_detect,
	// #ifndef ADD_I2C_DEVICE_ANDROID_4_0
	// .address_data = &addr_data,
	// #endif
};

int tpd_local_init(void)
{
	int retval;
//#if defined(GSL_GESTURE)   // added by xen for gesture flag, 20141013
    //struct proc_dir_entry *tp_gesture_proc = NULL;  // added by xen for gesture flag, 20141008
//#endif
	FUNC_ENTRY();

	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
		retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
		if (retval != 0) {
			GSL_DUBGE("gslX680  fail to set reg-vgp6 voltage: %d\n", retval);
			return -1;
		}


	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		GSL_DUBGE("gslX680  unable to add i2c driver.\n");
		return -1;
	}
	///*
	

	if(tpd_load_status == 0)
	{
		GSL_DUBGE("gslX680  add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	//*/
#ifdef TPD_HAVE_BUTTON
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())    // wanghe 2013-08-27
     	{
     GSL_DUBGE("xjl factory boot\n");
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local_factory, tpd_keys_dim_local);// initialize tpd button data
     	}else{
     GSL_DUBGE("xjl normal boot\n");
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

#if 0//defined(GSL_GESTURE)   // added by xen for gesture flag, 20141008
    tp_gesture_proc = proc_create(TP_GESTURE_PROC_FILE, 0666, NULL, NULL);

    if (tp_gesture_proc == NULL)
		GSL_DUBGE("create_proc_entry %s failed\n", TP_GESTURE_PROC_FILE);
#endif

	tpd_type_cap = 1;

	FUNC_EXIT();
	return 0;
}

/* Function to manage low power suspend */
void tpd_suspend(struct device *h)
{
	FUNC_ENTRY();

#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		return;
	}
#endif

#ifdef GSL_MONITOR
	GSL_DUBGE( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
	cancel_delayed_work_sync(&gsl_monitor_work);
	i2c_lock_flag = 0;  //added by xen 20141108
#endif

#ifdef GSL_GESTURE

	gsl_halt_flag = 1;
	if(gsl_gesture_flag == 1){
		u8 buf[4];
		//unsigned int temp;
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
	GSL_DUBGE( "gsl_ts_suspend () : cancel gsl_monitor_work\n");
	cancel_delayed_work_sync(&gsl_monitor_work);
#endif

	//mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	disable_irq(gsl_touch_irq);
	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	tpd_gpio_output(GTP_RST_PORT, 0);
}

/* Function to manage power-on resume */
void tpd_resume(struct device *h)
{
	
	FUNC_ENTRY();
#if 0 //cjc 20160201
	if (tpd_proximity_flag == 1)
	{
		tpd_enable_ps(1);
		return;
	}
#endif
	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
#ifdef GSL_GESTURE
	if(gsl_gesture_flag == 1){
		u8 buf[4];
		//unsigned int temp;
		//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		//msleep(20);
		//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		//msleep(5);
		tpd_gpio_output(GTP_RST_PORT, 0);
		msleep(20);
		tpd_gpio_output(GTP_RST_PORT, 1);
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
	}
	
	//start cty for test
	else
	{
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	enable_irq(gsl_touch_irq);
	}
		gsl_halt_flag = 0;
	//end cty for test	
		
#else
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);	
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	enable_irq(gsl_touch_irq);
#endif
		
	//start cty for test
	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);	
	//end cty for test	
	reset_chip(i2c_client);
	startup_chip(i2c_client);
	check_mem_data(i2c_client);	
#ifdef TPD_PROXIMITY //cjc 20160201
  if (tpd_proximity_flag == 1)
  {
    tpd_enable_ps(1);
    return;
  }
#endif
/*	
	tpd_halt = 0;
#ifdef GSL_GESTURE	
	gsl_halt_flag = 0;
#endif
*/
#ifdef GSL_MONITOR
	GSL_DUBGE( "gsl_ts_resume () : queue gsl_monitor_work\n");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
	i2c_lock_flag = 0;
#endif
	//start cty for test
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	tpd_halt = 0;
	//end cty for test	
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = GSLX680_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
/*#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif*/
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	FUNC_ENTRY();
	
// #ifdef ADD_I2C_DEVICE_ANDROID_4_0
// 	//i2c_register_board_info(0, &gslX680_i2c_tpd, 1);
// 	i2c_register_board_info(1, &gslX680_i2c_tpd, 1);  // changed by wanghe 2013-07-27	
// #endif


	tpd_get_dts_info();
	if(tpd_driver_add(&tpd_device_driver) < 0)
		GSL_DUBGE("add gslX680  driver failed\n");
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
	GSL_DUBGE("Sileadinc gslX680 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
//#endif


