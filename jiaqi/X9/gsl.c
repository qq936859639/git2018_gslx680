#if defined(__TOUCH_PANEL_CAPACITY__)
/**********************************************************************
*	Head files
**********************************************************************/
#include "kal_release.h"
// #include "pwic.h"
#include "sccb_v2.h"
#include "touch_panel.h"
#include "eint.h"
//#include "Capacitive_TP_Silead_GSL960.h"
#include "capacitive_tp_focaltech_GSL2236.h"
#include "drv_comm.h"
#include "Capacitive_TP_I2C.h"
#include "I2c.h"
#include "Dcl.h"
#include "touch_panel_custom.h"
extern void lzb_print(char *fmt, ...);
/*****************************************************
			CTP debug macro define
*****************************************************/
//#define DBG_CTP
#ifdef DBG_CTP
	#define ctp_dbg_print		kal_prompt_trace
#else
	#define ctp_dbg_print(...)
#endif

#ifdef DBG_CTP
	#define	M_dbg_print	dbg_print
#else
	#define	M_dbg_print(...) 
#endif

#define CTP_SLAVE_ADDR		0x80
#define MAX_FINGERS	  		5
#define MAX_CONTACTS		10
#define SMBUS_TRANS_LEN	0x20
#define GSL_PAGE_REG		0xf0
#define TD_STAT_NUMBER_TOUCH 0x07
#define POINTS_REG 0x80


/*****************************************************
			CTP global struct and variable
*****************************************************/
/*
The variable CTP_DELAY_TIME is to indicate the I2C transmission speed.
But the speed will be modified after calling function ctp_i2c_configure()
*/
kal_uint32 CTP_DELAY_TIME   = 6;

CTP_parameters_struct CTP_parameters;
DCL_HANDLE power_handle, reset_handle;
DCL_HANDLE sda_handle, scl_handle;
DCL_HANDLE eint_handle;
	  
extern const char gpio_ctp_power_enable_pin;
extern const char gpio_ctp_eint_pin;
extern const char gpio_ctp_eint_pin_M_EINT;

static kal_uint32 id_sign[MAX_CONTACTS+1] = {0};
static kal_uint8 id_state_flag[MAX_CONTACTS+1] = {0};
static kal_uint8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static kal_uint16 x_old[MAX_CONTACTS+1] = {0};
static kal_uint16 y_old[MAX_CONTACTS+1] = {0};
static kal_uint16 x_new = 0;
static kal_uint16 y_new = 0;



#define CTP_WRITE             				0x80
#define CTP_READ             					0x81
#define CTP_ACK_COUNTER        			10//150
#define CTP_I2C_DELAY_FT											50//200//500

extern const char gpio_ctp_reset_pin ;
#define CTP_RESET_PIN	gpio_ctp_reset_pin

extern const char gpio_ctp_i2c_scl_pin;
extern const char gpio_ctp_i2c_sda_pin;
#define CTP_I2C_DATA_PIN	gpio_ctp_i2c_sda_pin 
#define CTP_I2C_CLK_PIN		gpio_ctp_i2c_scl_pin 

#define OUTPUT 1
#define INPUT 0
#define CTP_SET_I2C_CLK_OUTPUT				GPIO_InitIO(OUTPUT,CTP_I2C_CLK_PIN)
#define CTP_SET_I2C_DATA_OUTPUT			GPIO_InitIO(OUTPUT,CTP_I2C_DATA_PIN)
#define CTP_SET_I2C_DATA_INPUT				GPIO_InitIO(INPUT,CTP_I2C_DATA_PIN)
#define CTP_SET_I2C_CLK_HIGH				GPIO_WriteIO(1,CTP_I2C_CLK_PIN)
#define CTP_SET_I2C_CLK_LOW				GPIO_WriteIO(0,CTP_I2C_CLK_PIN)

#define CTP_GET_I2C_CLK_BIT					GPIO_ReadIO(CTP_I2C_CLK_PIN)
#define SET_I2C_CLK_INPUT					GPIO_InitIO(INPUT,CTP_I2C_CLK_PIN)

#define CTP_SET_I2C_DATA_HIGH				GPIO_WriteIO(1,CTP_I2C_DATA_PIN)
#define CTP_SET_I2C_DATA_LOW				GPIO_WriteIO(0,CTP_I2C_DATA_PIN)
#define CTP_GET_I2C_DATA_BIT				GPIO_ReadIO(CTP_I2C_DATA_PIN)


#define CTP_SET_RESET_PIN_OUTPUT			GPIO_InitIO(OUTPUT, CTP_RESET_PIN)
#define CTP_SET_RESET_PIN_HIGH				GPIO_WriteIO(1, CTP_RESET_PIN)
#define CTP_SET_RESET_PIN_LOW				GPIO_WriteIO(0, CTP_RESET_PIN)


#define CTP_I2C_START_BIT \
	{ \
		volatile kal_uint32 j; \
		CTP_SET_I2C_CLK_OUTPUT; \
		CTP_SET_I2C_DATA_OUTPUT; \
		CTP_SET_I2C_CLK_HIGH; \
		CTP_SET_I2C_DATA_HIGH; \
		for(j=0;j<CTP_I2C_DELAY_FT;j++);\
		CTP_SET_I2C_DATA_LOW; \
		for(j=0;j<CTP_I2C_DELAY_FT;j++);\
		CTP_SET_I2C_CLK_LOW; \
	}

	#define CTP_I2C_STOP_BIT \
	{ \
		volatile kal_uint32 j; \
		CTP_SET_I2C_CLK_OUTPUT; \
		CTP_SET_I2C_DATA_OUTPUT; \
		CTP_SET_I2C_CLK_LOW; \
		CTP_SET_I2C_DATA_LOW; \
		for(j=0;j<CTP_I2C_DELAY_FT;j++);\
		CTP_SET_I2C_CLK_HIGH; \
		for(j=0;j<CTP_I2C_DELAY_FT;j++);\
		CTP_SET_I2C_DATA_HIGH; \
		for(j=0;j<CTP_I2C_DELAY_FT;j++);\
	}

void CTP_I2C_Init(void)
{
	GPIO_ModeSetup(CTP_I2C_CLK_PIN, 0);
	GPIO_ModeSetup(CTP_I2C_DATA_PIN, 0);
	GPIO_ModeSetup(CTP_RESET_PIN, 0);
	CTP_SET_I2C_CLK_OUTPUT;
	CTP_SET_I2C_DATA_OUTPUT;


	CTP_SET_I2C_CLK_HIGH;
	CTP_SET_I2C_DATA_HIGH;
	CTP_SET_RESET_PIN_OUTPUT;
}
void CTP_delay_us_ft6206(kal_uint32 time)
{
	kal_uint32 i,j,k;
	for(i=0;i< time;i++)
		for(j=0;j<100;j++)
          {
            k=0;
        };
}
static void CTP_delay_ms(kal_uint32 time)
{//zhouwei add
    volatile kal_uint32 delay;
    while(time--)	
    for (delay =0;delay <95000;delay++) {} // 1ms
}
kal_uint8 CTP_I2C_send_byte(kal_uint8 send_byte)

{
	volatile signed char i;
	volatile kal_uint32 j;
	volatile kal_uint32 k=0;
	kal_uint8 ack;
	
	for (i=7;i>=0;i--)
	{	/* data bit 7~0 */
		if (send_byte & (1<<i))
		{
			CTP_SET_I2C_DATA_HIGH;

		}
		else
		{
			CTP_SET_I2C_DATA_LOW;

		}
		for(j=0;j<CTP_I2C_DELAY_FT;j++);
		CTP_SET_I2C_CLK_HIGH;
		for(j=0;j<CTP_I2C_DELAY_FT;j++);
		CTP_SET_I2C_CLK_LOW;
		for(j=0;j<CTP_I2C_DELAY_FT;j++);
	}
	
	CTP_SET_I2C_DATA_INPUT;
	CTP_SET_I2C_CLK_HIGH;
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	
	for(k=0;k<CTP_ACK_COUNTER;k++)
	{
		if(CTP_GET_I2C_DATA_BIT==0)
		{
			ack=1;
			break;
		}
		else
		{
			ack=0;
		}
	}

	CTP_SET_I2C_CLK_LOW;
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_DATA_OUTPUT;
	
	//CTP_delay_us_ft6206(100);//need delay can't delete
	CTP_delay_us_ft6206(6);
	//dbg_print("\n\r zs2010 CTP_I2C_send_byte ack = %d\r\n",ack);
	return ack;
}	

kal_uint8 CTP_I2C_get_byte(void)
{
	volatile signed char i;
	volatile kal_uint32 j;
	kal_uint8 get_byte=0;

	CTP_SET_I2C_DATA_INPUT;
	  for (i=7;i>=0;i--)
        {       /* data bit 7~0 */
                CTP_SET_I2C_CLK_LOW;
                for(j=0;j<CTP_I2C_DELAY_FT;j++);
                CTP_SET_I2C_CLK_HIGH;
                for(j=0;j<CTP_I2C_DELAY_FT;j++);
                if (CTP_GET_I2C_DATA_BIT)
                        get_byte |= (1<<i);
        }
	/* don't care bit, 9th bit */
	
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_CLK_LOW;
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_DATA_OUTPUT;
	CTP_delay_us_ft6206(5);
	return get_byte;
}	


kal_bool gsl_write_bytes(kal_uint8 reg_addr, kal_uint8 *data, kal_uint8 len)
{
	volatile kal_uint32 j;
	kal_uint32 c;
	
	CTP_I2C_START_BIT;		 
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(CTP_WRITE))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(reg_addr))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);


	for(c=0;c<len;c++)
		{
			if(!CTP_I2C_send_byte(*(data+c)))
			return(-1);
		}

	
		for(j=0;j<CTP_I2C_DELAY_FT;j++);
	   CTP_I2C_STOP_BIT;	  
	   return 1;

}


kal_int8 CTP_I2C_WRITE(kal_uint8 reg, kal_uint8 value)
{
	volatile kal_uint32 j;
	
	CTP_I2C_START_BIT;       
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(CTP_WRITE))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(reg))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(value))
	return(-1);
    	for(j=0;j<CTP_I2C_DELAY_FT;j++);
       CTP_I2C_STOP_BIT;      
       return 1;

}

void CTP_BL_I2C_WRITE(data, len)
{
	kal_uint16 i;
	//i2c_write(SCCB_OWNER_TP, data, len);
}


kal_uint8 CTP_I2C_get_byte_with_ack(unsigned char uc_ack_lvl)
{
	volatile signed char i;
	volatile kal_uint32 j;
	kal_uint8 get_byte=0;

	CTP_SET_I2C_DATA_INPUT;
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	  for (i=7;i>=0;i--)
        {       /* data bit 7~0 */
                CTP_SET_I2C_CLK_LOW;
                for(j=0;j<CTP_I2C_DELAY_FT;j++);
                CTP_SET_I2C_CLK_HIGH;
                for(j=0;j<CTP_I2C_DELAY_FT;j++);
                if (CTP_GET_I2C_DATA_BIT)
                        get_byte |= (1<<i);
				for(j=0;j<CTP_I2C_DELAY_FT;j++);
        }
	/* don't care bit, 9th bit */

	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_CLK_LOW;
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_DATA_OUTPUT;
	
	if (uc_ack_lvl == 1)
	{
	    CTP_SET_I2C_DATA_HIGH;
	}
	else
	{
	    CTP_SET_I2C_DATA_LOW;
	}
	
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_CLK_HIGH;

	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_CLK_LOW;

	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_SET_I2C_DATA_LOW;
	
	//CTP_delay_us_ft6206(5);
	return get_byte;
}	



kal_int8 gsl_read_bytes(kal_uint8 reg, kal_uint8 *value, kal_uint16 len) 
{

	volatile kal_uint32 j;
    kal_uint8 get_byte = 0;
	kal_uint16 i;
	
    CTP_I2C_START_BIT;       
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(CTP_WRITE))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(reg))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);

	  
    CTP_I2C_STOP_BIT;
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_I2C_START_BIT;       
	if(!CTP_I2C_send_byte(CTP_READ))
	return(-1);

	if (len <= 1)
    {
        //*value =CTP_I2C_get_byte();
        *value =CTP_I2C_get_byte_with_ack(1);
		for(j=0;j<CTP_I2C_DELAY_FT;j++);
    }
	else
	{
	    for (i = 0; i< len - 1; i++)
	    {
	        *value++ =CTP_I2C_get_byte_with_ack(0);
	    }
		*value =CTP_I2C_get_byte_with_ack(1);
	}
       CTP_I2C_STOP_BIT;
 	 
       return 1;
}

kal_int8 CTP_I2C_READ(kal_uint8 reg, kal_uint8 *value, kal_uint16 len) 
{

	volatile kal_uint32 j;
    kal_uint8 get_byte = 0;
	kal_uint16 i;
	
    CTP_I2C_START_BIT;       
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(CTP_WRITE))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	if(!CTP_I2C_send_byte(reg))
	return(-1);
	for(j=0;j<CTP_I2C_DELAY_FT;j++);

       CTP_I2C_STOP_BIT;
	for(j=0;j<CTP_I2C_DELAY_FT;j++);
	CTP_I2C_START_BIT;       
	if(!CTP_I2C_send_byte(CTP_READ))
	return(-1);

	if (len <= 1)
    {
        //*value =CTP_I2C_get_byte();
        *value =CTP_I2C_get_byte_with_ack(1);
		for(j=0;j<CTP_I2C_DELAY_FT;j++);
    }
	else
	{
	    for (i = 0; i< len - 1; i++)
	    {
	        *value++ =CTP_I2C_get_byte_with_ack(0);
	    }
		*value =CTP_I2C_get_byte_with_ack(1);
	}
       CTP_I2C_STOP_BIT;
 	 
       return 1;
}

/*********************************************************************
 *    			I2C and CTP transmission API
 *				Function Implement
 *********************************************************************/
/*
Enable HW I2C power domain.
JUST for HW I2C.

CTP_I2C_LDO 			==>		HW I2C using LDO 	(see to enum PMU_LDO_BUCK_LIST_ENUM)
CTP_I2C_LDO_VOLTAGE	==>		The voltage of VIO	(see to enum PMU_VOLTAGE_ENUM)
CTP_I2C_LDO and CTP_I2C_LDO_VOLTAGE are defined in the file "touch_panel_custom.h".
The customer can modify them according to the actual hardware design
*/
void CTP_I2C_POWER_ON(void)
{
	ctp_i2c_power_on(KAL_TRUE, CTP_I2C_LDO, CTP_I2C_LDO_VOLTAGE);
}

static void startup_chip(void)
{
	kal_uint8 write_buf = 0x00;
      kal_uint8 buf[4] = {0};
	
	write_buf= 0x04;
	gsl_write_bytes( 0xe4, &write_buf, 1); 	
	CTP_delay_ms(2);
	write_buf= 0x0;	
	gsl_write_bytes( 0xe0, &write_buf, 1); 
	#ifdef GSL_NOID_VERSION
    #ifdef GSL_IDENTY_TP_GSLXX3X_COMPRESS
		if (gsl_tp_type > 0)
    #endif
	gsl_DataInit(gsl_config_data_id);
ctp_dbg_print(MOD_TP_TASK, "get  over......\r\n");
    
#endif	
	CTP_delay_ms(2);		
}

static void reset_chip(void)
{
	kal_uint8 write_buf[4]	= {0};

	write_buf[0] = 0x88;
	gsl_write_bytes( 0xe0, &write_buf[0],1); 	
	//kal_sleep_task(10);
	CTP_delay_ms(16);

	write_buf[0] = 0x04;
	gsl_write_bytes( 0xe4, &write_buf[0], 1); 	
	CTP_delay_ms(1);

	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;
	write_buf[3] = 0x00;
	gsl_write_bytes( 0xbc, write_buf, 4); 	
	CTP_delay_ms(2);
}

static void clr_reg(void)
{
	kal_uint8 write_buf[4]	= {0};

	write_buf[0] = 0x88;
	gsl_write_bytes(0xe0, &write_buf[0], 1); 	
	//kal_sleep_task(20);
	CTP_delay_ms(16);
	write_buf[0] = 0x03;
	gsl_write_bytes(0x80, &write_buf[0], 1); 	
	CTP_delay_ms(5);
	write_buf[0] = 0x04;
	gsl_write_bytes(0xe4, &write_buf[0], 1); 	
	CTP_delay_ms(1);
	write_buf[0] = 0x00;
	gsl_write_bytes(0xe0, &write_buf[0], 1); 	
	CTP_delay_ms(2);
}
#ifdef GSL_IDENTY_TP_GSLXX3X_COMPRESS
static void gsl_load_fw(struct fw_data *GSL_DOWNLOAD_DATA, unsigned int source_len) 
{
	kal_uint8 buf[SMBUS_TRANS_LEN*4] = {0};
	kal_uint8 reg = 0, send_offset = 0, cur = 0;
	unsigned int source_line = 0;

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		if(0 == (source_line%GSL_PAGE_SIZE)){
			reg = GSL_PAGE_REG;
			buf[0] = (char)(GSL_DOWNLOAD_DATA[source_line].val & 0x000000ff);
			gsl_write_bytes(GSL_PAGE_REG, &buf[0], 1);
			continue;
		}else if(1 == (source_line%GSL_PAGE_SIZE))
			cur = 0;
		else
			cur++;
		send_offset =  (cur%(SMBUS_TRANS_LEN < 0x20 ? SMBUS_TRANS_LEN : 0x20));
		if (0 == send_offset)
			reg = cur*GSL_UNIT_SIZE;			
		buf[send_offset*GSL_UNIT_SIZE + 0] = (char)(GSL_DOWNLOAD_DATA[source_line].val & 0x000000ff);
		buf[send_offset*GSL_UNIT_SIZE + 1] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0x0000ff00) >> 8);
		buf[send_offset*GSL_UNIT_SIZE + 2] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0x00ff0000) >> 16);
		buf[send_offset*GSL_UNIT_SIZE + 3] = (char)((GSL_DOWNLOAD_DATA[source_line].val & 0xff000000) >> 24);
		if (send_offset == ((SMBUS_TRANS_LEN < 0x20 ? SMBUS_TRANS_LEN : 0x20)-1))
			gsl_write_bytes(reg, buf, SMBUS_TRANS_LEN * GSL_UNIT_SIZE);
	}
}
#else
static void gsl_load_fw(void)
{
	kal_uint8 buf[SMBUS_TRANS_LEN*4] = {0};
	kal_uint8 reg = 0, send_flag = 1, cur = 0;
	
	unsigned int source_line = 0;
	unsigned int source_len = sizeof(GSLX680_FW)/sizeof(GSLX680_FW[0]);

//	ctp_dbg_print(MOD_TP_TASK, "=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
        /* if (8 == SMBUS_TRANS_LEN)
		{
			reg = GSLX680_FW[source_line].offset;

			buf[0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
			buf[1] = (char)((GSLX680_FW[source_line].val & 0x0000ff00) >> 8);
			buf[2] = (char)((GSLX680_FW[source_line].val & 0x00ff0000) >> 16);
			buf[3] = (char)((GSLX680_FW[source_line].val & 0xff000000) >> 24);

			gsl_write_bytes( reg, buf, 4); 	
		}
		 else*/
		 	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == GSLX680_FW[source_line].offset)
		{
			buf[0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
			gsl_write_bytes( GSL_PAGE_REG, &buf[0], 1); 	
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
				reg = GSLX680_FW[source_line].offset;

			buf[cur + 0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
			buf[cur + 1] = (char)((GSLX680_FW[source_line].val & 0x0000ff00) >> 8);
			buf[cur + 2] = (char)((GSLX680_FW[source_line].val & 0x00ff0000) >> 16);
			buf[cur + 3] = (char)((GSLX680_FW[source_line].val & 0xff000000) >> 24);
			cur += 4;

			if (0 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08)) 
			{
				gsl_write_bytes( reg, buf, SMBUS_TRANS_LEN*4); 	
				cur = 0;
			}

			send_flag++;

		}
		 	}
	}

	//ctp_dbg_print(MOD_TP_TASK, "=============gsl_load_fw end==============\n");

}

#endif


static int _GSLX680_test_i2c(void)
{
	int rc = 1;
	kal_uint8 addr;
	kal_uint8 buf;

	addr = 0xf0;

	if(!gsl_read_bytes(addr, &buf,1) )
		rc --;
	//kal_prompt_trace(MOD_TP_TASK, "[TP]_GSLX680_test_i2c read 0xf0 = %x", buf);
	dbg_print("[TP]_GSLX680_test_i2c read 0xf0 = %x\r\n", buf);
	buf = 0x12;
       gsl_write_bytes(addr, &buf,1);
	CTP_delay_ms(2);
	//kal_prompt_trace(MOD_TP_TASK, "[TP]_GSLX680_test_i2c write 0xf0 = %x", buf);
	dbg_print("[TP]_GSLX680_test_i2c read11 0xf0 = %x\r\n", buf);

	if(!gsl_read_bytes(addr, &buf,1) )
		rc --;
	//kal_prompt_trace(MOD_TP_TASK, "[TP]_GSLX680_test_i2c read 0xf0 = %x", buf);
	
	dbg_print("[TP]_GSLX680_test_i2c read1122 0xf0 = %x\r\n", buf);
	CTP_delay_ms(2);
	return rc;
}

#ifdef GSL_IDENTY_TP_GSLXX3X_COMPRESS

int gsl_get_min(int a,int b)
{
	return (a<=b) ? a : b;
}
int gsl_reg_compare(kal_uint32 sample, kal_uint32 compare)
{
	char tmp1;
	int i;
	int result = 0;
	kal_uint32 filler = 0x01;
	for (i=0; i<length_reg; i++) {
		if((sample & (filler << i)) && (!(compare & (filler << i))))
				result++;
		else if (!(sample & (filler << i))  && ( (compare & (filler << i)))) 
				result += length_reg;
	}
	return result;
}
static void gsl_identify_tp()
{
	kal_uint8 addr = 0x00;
	kal_uint8 write_buf = 0x00;
	kal_uint8 read_b4[4] = {0x00};
	kal_uint8 read_b8[4] = {0x00};
	kal_uint8 read_ac[4] = {0x00};
	int i;
	unsigned int fw_len = 0;
	kal_uint32 tmp_b8 = 0x00000000;
	kal_uint32 tmp_ac = 0x00000000;
	for (i = 0; i < 8; i++ ) {
		CTP_SET_RESET_PIN_LOW;
		CTP_delay_ms(10);
		CTP_SET_RESET_PIN_HIGH;
		clr_reg();
		reset_chip();
		fw_len = sizeof(GSL_TP_CHECK_FW)/sizeof(GSL_TP_CHECK_FW[0]);
		gsl_load_fw(GSL_TP_CHECK_FW, fw_len);
		gsl_write_bytes( 0xe0, &write_buf, 1);
		CTP_delay_ms(200);
		addr = 0xb4;
		gsl_read_bytes(addr, read_b4,4);
                CTP_delay_us_ft6206(6);
		gsl_read_bytes(addr, read_b4,4);
                CTP_delay_us_ft6206(6);
		if(!read_b4[0]&&!read_b4[1]&&!read_b4[2]&&!read_b4[3]){
			kal_prompt_trace(MOD_TP_TASK, "[TP]gsl_identify_tp read 0xb4 = 0x%02x%02x%02x%02x", read_b4[3],read_b4[2],read_b4[1],read_b4[0]);
			id_type_err++;
			continue;
		}
		addr = 0xb8;
		gsl_read_bytes(addr, read_b8,4);
                CTP_delay_us_ft6206(6);
		gsl_read_bytes(addr, read_b8,4);
                CTP_delay_us_ft6206(6);
		tmp_b8 = (read_b8[3]<<24)|(read_b8[2]<<16)|(read_b8[1]<<8)|read_b8[0];
		kal_prompt_trace(MOD_TP_TASK, "[TP]gsl_identify_tp read 0xb8 = 0x%02x%02x%02x%02x", read_b8[3],read_b8[2],read_b8[1],read_b8[0]);
		addr = 0xac;
		gsl_read_bytes(addr, read_ac,4);
		gsl_read_bytes(addr, read_ac,4);
		tmp_ac = (read_ac[3]<<24)|(read_ac[2]<<16)|(read_ac[1]<<8)|read_ac[0];
		kal_prompt_trace(MOD_TP_TASK, "[TP]gsl_identify_tp read 0xac = 0x%02x%02x%02x%02x", read_ac[3],read_ac[2],read_ac[1],read_ac[0]);
		tmp0_result = 0;
		tmp1_result = 0;
		tmp2_result = 0;
		tmp3_result = 0;
		tmp1_result = gsl_reg_compare(FRIST_B8, tmp_b8);
		tmp1_result += gsl_reg_compare(FRIST_AC, tmp_ac);
		tmp2_result = gsl_reg_compare(SECOND_B8, tmp_b8);
		tmp2_result += gsl_reg_compare(SECOND_AC, tmp_ac);
		tmp3_result = gsl_reg_compare(THIRD_B8, tmp_b8);
		tmp3_result += gsl_reg_compare(THIRD_AC, tmp_ac);
		tmp0_result = gsl_get_min(tmp1_result, tmp2_result);
		tmp0_result = gsl_get_min(tmp0_result, tmp3_result);
        kal_prompt_trace(MOD_TP_TASK, "tmp0_result=%d,tmp1_result=%d,tmp2_result=%d,tmp3_result=%d",tmp0_result,tmp1_result,tmp2_result,tmp3_result);
		if (tmp0_result == tmp1_result){
			kal_prompt_trace(MOD_TP_TASK, "[TP]gsl_identify_tp TP change 1st TP.");
			gsl_tp_type = 1;	/* 1st tp */
			break;
		} else if (tmp0_result == tmp2_result) {
			kal_prompt_trace(MOD_TP_TASK, "[TP]gsl_identify_tp TP change 2nd TP.");
			gsl_tp_type = 2;	/* 2nd tp */
			break;
		} else if (tmp0_result == tmp3_result) {
			kal_prompt_trace(MOD_TP_TASK, "[TP]gsl_identify_tp TP change 3th TP.");
			gsl_tp_type = 3;	/* 3th tp */
			break;
		} 
	}
}
#endif
static void init_chip(void)
{
	int rc;
	
#ifdef GSL_IDENTY_TP_GSLXX3X_COMPRESS
		unsigned int fw_len;
#endif
	
	
	//rc = _GSLX680_test_i2c();
	//if(rc < 0)
	//	kal_prompt_trace(MOD_TP_TASK, "------gslX680 test_i2c error------");

	clr_reg();
	reset_chip();
#ifdef GSL_IDENTY_TP_GSLXX3X_COMPRESS
		gsl_identify_tp();
		if	(1 == gsl_tp_type) { // 1st tp
			fw_len = sizeof(GSLX680_FW_1)/sizeof(GSLX680_FW_1[0]);
			gsl_load_fw(GSLX680_FW_1, fw_len);
                        fw_len = sizeof(GSLX680_FW_COMM)/sizeof(GSLX680_FW_COMM[0]);
			gsl_load_fw(GSLX680_FW_COMM, fw_len);
			gsl_config_data_id = gsl_config_data_id_1;
		} else if (2 == gsl_tp_type) { // 2nd tp
			fw_len = sizeof(GSLX680_FW_2)/sizeof(GSLX680_FW_2[0]);
			gsl_load_fw(GSLX680_FW_2, fw_len);
                        fw_len = sizeof(GSLX680_FW_COMM)/sizeof(GSLX680_FW_COMM[0]);
			gsl_load_fw(GSLX680_FW_COMM, fw_len);
			gsl_config_data_id = gsl_config_data_id_2;
		}else if (3 == gsl_tp_type) { // 2nd tp
			fw_len = sizeof(GSLX680_FW_2)/sizeof(GSLX680_FW_2[0]);
			gsl_load_fw(GSLX680_FW_2, fw_len);
                        fw_len = sizeof(GSLX680_FW_COMM)/sizeof(GSLX680_FW_COMM[0]);
			gsl_load_fw(GSLX680_FW_COMM, fw_len);
			gsl_config_data_id = gsl_config_data_id_2;
		}else {
			fw_len = sizeof(GSLX680_FW_1)/sizeof(GSLX680_FW_1[0]);
			gsl_load_fw(GSLX680_FW_1, fw_len);
                        fw_len = sizeof(GSLX680_FW_COMM)/sizeof(GSLX680_FW_COMM[0]);
			gsl_load_fw(GSLX680_FW_COMM, fw_len);
			gsl_config_data_id = gsl_config_data_id_1;
			gsl_tp_type = 1;
		}
#else
	gsl_load_fw();
#endif
	startup_chip();
	reset_chip();			
	startup_chip();
}

static void check_mem_data(void)
{
	kal_uint8 read_buf[4]  = {0};
	int i;
	CTP_delay_ms(30);
	gsl_read_bytes(0xb0, read_buf, sizeof(read_buf));
	//ctp_dbg_print(MOD_TP_TASK, "#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	
	dbg_print("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
/*for(i=0;i<10;i++)
	{gsl_read_bytes(0xb4, read_buf, sizeof(read_buf));
	ctp_dbg_print(MOD_TP_TASK, "#########check mem read 0xb4 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
    }
*/
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		kal_prompt_trace(MOD_CC,"#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		ctp_dbg_print(MOD_TP_TASK, "#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip();
	}
}

/*
Upper layer will call this function to enable/disable CTP power.
If FOCUS the POWRE CONSUME, it is suggested to implement it!
*/
kal_bool ctp_gslX680_get_data();
kal_bool ctp_gslX680_init();
void ctp_gslX680_power_on(kal_bool enable)
{
	//_TODO:  Implement this funciton by customer

	static kal_bool power_status = KAL_FALSE;
	kal_bool wakeup, sleep;

	kal_uint8 write_data[2];
	kal_bool temp_result = KAL_TRUE;
	dbg_print("ctp_gslX680_power_on enable=%d\r\n",enable);
	if (enable == power_status)	//do not turn on/off pmu ldo again.
		return;
	power_status = enable;

	if (enable)
	{

	   CTP_SET_RESET_PIN_HIGH;
		//CTP_I2C_Init();
		CTP_delay_ms(30);
		reset_chip();			
		startup_chip();	
		check_mem_data();
		//ctp_dbg_print("ctp on........\r\n");
		//init_chip();	// wuxi ++ 1604060900
		ctp_dbg_print(MOD_TP_TASK, "ctp on........");
		//_GSLX680_test_i2c();
	}
	else
	{

		//_GSLX680_test_i2c();
	  CTP_SET_RESET_PIN_LOW;	
		//ctp_dbg_print("ctp off........\r\n");
		ctp_dbg_print(MOD_TP_TASK, "ctp off........");
		
		dbg_print("ctp off........\r\n");
	}
/*	
	//DclGPIO_Close(power_handle);
	DclGPIO_Close(reset_handle);

	kal_sleep_task(30);
	DclGPIO_Control(reset_handle, GPIO_CMD_WRITE_LOW, NULL);
	kal_sleep_task(20);
	DclGPIO_Control(reset_handle, GPIO_CMD_WRITE_HIGH, NULL);
	kal_sleep_task(20); 
*/
}


/***************************************************************
*    			     GSL CTP driver API
*				Function Implement
***************************************************************/
/*
GSL CTP initialization

In the initialization, we should do:
1. Enable VIO power domain and config the slaver address and i2c speed
2. Config EINT debounce time and sensitivity
*/
Touch_Panel_PenState_enum ctp_gslX680_hisr(void);
kal_bool ctp_gslX680_init(void)
{
	kal_uint8 lvalue;
	kal_uint8 write_data[2];
	kal_bool temp_result = KAL_TRUE;
	DCL_HANDLE reset_handle;
	DCL_HANDLE eint_handle;
      //ctp_dbg_print("ctp2 .................................................................................\r\n");
		ctp_dbg_print(MOD_TP_TASK, "ctp 2........");
	  
	ctp_gslX680_power_on(KAL_TRUE);
	
	init_chip();
	//check_mem_data();
//	ctp_dbg_print("ctp .................................................................................\r\n");
		ctp_dbg_print(MOD_TP_TASK, "ctp........");
	/*
	config EINT debounce time and sensitivity
	MUST set 0 to EINT debounce
	*/
	#if 0
	eint_handle = DclGPIO_Open(DCL_GPIO, gpio_ctp_eint_pin);
	
    DclGPIO_Control(eint_handle, GPIO_CMD_SET_DIR_IN, NULL);
    DclGPIO_Control(eint_handle, gpio_ctp_eint_pin_M_EINT, NULL);	
    EINT_Set_HW_Debounce(custom_eint_get_channel(touch_panel_eint_chann), 2);//sungang
    EINT_SW_Debounce_Modify(custom_eint_get_channel(touch_panel_eint_chann),0);//sungang
	  EINT_Set_Sensitivity(custom_eint_get_channel(touch_panel_eint_chann), EDGE_SENSITIVE);

	kal_sleep_task(1);
	#endif

	#if 1

	
	EINT_Set_HW_Debounce(custom_eint_get_channel(touch_panel_eint_chann), 2);
	EINT_SW_Debounce_Modify(custom_eint_get_channel(touch_panel_eint_chann),0);
	EINT_Set_Sensitivity(custom_eint_get_channel(touch_panel_eint_chann), EDGE_SENSITIVE);

	 //   EINT_Registration(custom_eint_get_channel(touch_panel_eint_chann),KAL_FALSE,0,ctp_gslX680_hisr, KAL_TRUE);
#endif
	return KAL_TRUE;
}

/*
If needed, uppder layer will call this function to swicth CTP mode.

By now, Maybe this function has not been used by upper layer.
If you want to implement this function, you can see the sample code in the function body.
*/
kal_bool ctp_gslX680_device_mode(ctp_device_mode_enum mode)
{
	return KAL_TRUE;
}

/*
The following function is JUST used in timer trigger mode.
Now we use interrupt mode to get data. So this function is not used.

ATTENTION: DO NOT delete this function!!
This function is one member of the CTP function pointer struct.
*/
Touch_Panel_PenState_enum ctp_gslX680_hisr(void)
{
	kal_bool temp_result;
	kal_uint8 lvalue[4];
	kal_uint32 model = 0;

	temp_result = gsl_read_bytes(POINTS_REG, lvalue, 4);
	model = (kal_uint32)((lvalue[0] & 0xf) & TD_STAT_NUMBER_TOUCH);
	
	if (model)
	{
		//ctp_dbg_print( "down\n");
		ctp_dbg_print(MOD_TP_TASK, "down........");
		return DOWN;
	}
	else
	{
		//ctp_dbg_print( "up\n");
		ctp_dbg_print(MOD_TP_TASK, "up........");
		return UP;
	}
}

/*
This function is used to get parameter from CTP IC or set parameter to CTP IC.

By now, Maybe this function has not been used by upper layer.
ATTENTION: If need to implement it, DO NOT get and set the same parameter in the same time
*/
kal_bool ctp_gslX680_parameters(CTP_parameters_struct *para, kal_uint32 get_para, kal_uint32 set_para)
{
	return 0;
}

/*
This function is a INTERNAL FUNCTION in the CTP driver.
It ONLY can be used to read the coordination value of one point
and JUST be called by gsl_read_all_point() function.
*/
kal_bool gsl_read_one_point(kal_uint32 x_base, TP_SINGLE_EVENT_T *event)
{
	kal_bool temp_result;
	kal_uint8 values[4] = {0};
	ctp_dbg_print(MOD_TP_TASK, "get  one......");

	temp_result = gsl_read_bytes(x_base, values, 4);

	if (KAL_FALSE == temp_result)
	{
		ctp_dbg_print(MOD_TP_TASK, "ctp_read_one_point fail!");
		ctp_dbg_print(MOD_TP_TASK, "ctp_read_one_point fail......");
		
		return KAL_FALSE;
	}
	else
	{
		event->z =   (kal_uint16)(values[3]&0xf0)>>8;
		event->x =   (((kal_uint16)(values[3]&0x0f))<<8) | values[2];
		event->y =   (((kal_uint16)(values[1]))<<8) | values[0];
		if(event->y>10000)
			return KAL_FALSE;
			
		return KAL_TRUE;
	}
}

/*
This fucntion is a INTERNAL FUNCTION in the CTP driver.

It ONLY can be used to read the coordination values of all points that are pressed now
and JUST be called by ctp_gslX680_get_data() function.
*/

kal_bool gsl_read_all_point(TouchPanelMultipleEventStruct *tpes, kal_uint32 points)
{
	kal_bool temp_result;
	kal_uint32 i = 0;

	TP_SINGLE_EVENT_T get_one_point;


#ifdef GSL_NOID_VERSION
	struct gsl_touch_info cinfo={0};
	kal_uint8 lvalue[4];
	kal_bool result;
	kal_uint32 tmp1 = 0;
	kal_uint8 buf[4] = {0};
	kal_uint32 version = gsl_version_id();
	ctp_dbg_print(MOD_TP_TASK, "[tp-gsl] version=0x%x", version);	
#endif
	ctp_dbg_print(MOD_TP_TASK, "get  all......");
	ctp_dbg_print(MOD_TP_TASK, "ctp_read_all_point points=%d", points);

	ASSERT(tpes);

	/*By now we ONLY can support FIVE points at most*/
	if (points > MAX_FINGERS)
	{
		return KAL_FALSE;
	}

#ifdef GSL_NOID_VERSION
	result = gsl_read_bytes(POINTS_REG, lvalue, 4);
	if(KAL_FALSE == result)
	{
		ctp_dbg_print(MOD_TP_TASK, "tp-gsl  read point_reg number failed");
		return KAL_FALSE;
	}
	cinfo.finger_num =  points;
	ctp_dbg_print(MOD_TP_TASK, "tp-gsl  finger_num = %d",cinfo.finger_num);
#endif

	for (i=0; i<points; i++)
	{
		temp_result = gsl_read_one_point(POINTS_REG + 4*(1+ i), &get_one_point);

		if (KAL_FALSE == temp_result)
		{
			//ctp_dbg_print(MOD_TP_TASK, "read %d point failed!");
			ctp_dbg_print(MOD_TP_TASK, "read all point failed!....");
			return KAL_FALSE;
		}
		else
		{
#ifdef GSL_NOID_VERSION
			cinfo.x[i] = get_one_point.x;
			cinfo.y[i] = get_one_point.y;
			cinfo.id[i] = get_one_point.z; 
		       //ctp_dbg_print(MOD_TP_TASK,  "piont[%d], x:%d, y:%d", i, get_one_point.x, get_one_point.y);
                        kal_prompt_trace(MOD_TP_TASK,  "piont[%d], x:%d, y:%d", i, get_one_point.x, get_one_point.y);
#else
			tpes->points[i].x = get_one_point.x;
			tpes->points[i].y = get_one_point.y;
			tpes->points[i].z = 0;
			ctp_dbg_print( "piont[%d], x:%d, y:%d", i, get_one_point.x, get_one_point.y);
#endif
		}
	}
#ifdef GSL_NOID_VERSION
	cinfo.finger_num = (lvalue[3]<<24)|(lvalue[2]<<16)|(lvalue[1]<<8)|lvalue[0];
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	ctp_dbg_print(MOD_TP_TASK, "[tp-gsl] tmp1=%x",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
		gsl_write_bytes( 0xf0, buf, 4); 
		buf[0]=(kal_uint8)(tmp1 & 0xff);
		buf[1]=(kal_uint8)((tmp1>>8) & 0xff);
		buf[2]=(kal_uint8)((tmp1>>16) & 0xff);
		buf[3]=(kal_uint8)((tmp1>>24) & 0xff);
		ctp_dbg_print(MOD_TP_TASK, "tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x", tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_write_bytes(0x8, buf, 4); 
	}
	if(cinfo.finger_num == 0)
	{		
		tpes->points[0].x = x_old[i];
		tpes->points[0].y = y_old[i];
		tpes->points[i].z = 0;
	}
	for (i=0; i<cinfo.finger_num; i++)
	{
		#if defined(__SUPPORT_GT08_STYLE__) && defined(__CAMERA_AT_BOTTOM__)
		tpes->points[i].x =  UI_DEVICE_WIDTH-cinfo.x[i];
		tpes->points[i].y =  UI_DEVICE_HEIGHT-cinfo.y[i];
		#else
		tpes->points[i].x =  cinfo.x[i];
		tpes->points[i].y =  cinfo.y[i];
		#endif
		x_old[i] = cinfo.x[i];
		y_old[i] = cinfo.y[i];
		tpes->points[i].z = 0;
		//ctp_dbg_print(MOD_TP_TASK,  "piont[%d], x:%d, y:%d", i, get_one_point.x, get_one_point.y);
		//ctp_dbg_print(MOD_TP_TASK, "tpes->points[%d], x:%d, y:%d", i, tpes->points[i].x, tpes->points[i].y);
                 kal_prompt_trace(MOD_TP_TASK,  "piont[%d], x:%d, y:%d", i, get_one_point.x, get_one_point.y);
                 kal_prompt_trace(MOD_TP_TASK, "tpes->points[%d], x:%d, y:%d", i, tpes->points[i].x, tpes->points[i].y);
	}
#endif
	ctp_dbg_print(MOD_TP_TASK, "xxxxxxxx");

	return KAL_TRUE;
}

/*
This function is used to get the raw data of the fingures that are pressed.
When CTP IC send intterupt signal to BB chip, this function will be called in the interrupt handler function.

ATTENTION: Becasue this function is called in the interrupt handler function, it MUST NOT run too long.
That will block the entire system.
If blocking too long, it generally will cause system crash *....*
*/
kal_bool ctp_gslX680_get_data(TouchPanelMultipleEventStruct *tpes)
{
	kal_bool temp_result;
	kal_uint8 lvalue[4];
	kal_uint32 counter = 0;
	kal_uint32 model = 0;
	ctp_dbg_print(MOD_TP_TASK, "get  data22......");

	ASSERT(tpes);

	tpes->time_stamp = (kal_uint16)L1I_GetTimeStamp();
	tpes->padding = CTP_PATTERN;

	temp_result = gsl_read_bytes(POINTS_REG, lvalue, 4);
	model = (kal_uint32)((lvalue[0] & 0xf) & TD_STAT_NUMBER_TOUCH);
	tpes->model = (kal_uint16)model;

	/*
	0 fingure meas UP EVENT, so return FALSE;
	And now we only support FIVE fingures at most, so if more than 5 fingures also return FALSE
	*/
	if ((model == 0)||(model > MAX_FINGERS))
	{
		ctp_dbg_print(MOD_TP_TASK, "model = %d", model);

		return KAL_FALSE;
	}

	temp_result = gsl_read_all_point(tpes, model);

	if (KAL_FALSE == temp_result)
	{
		ctp_dbg_print(MOD_TP_TASK, "read all points failed!");
		ctp_dbg_print(MOD_TP_TASK, "read all points failed!");

		return KAL_FALSE;
	}	

	return KAL_TRUE;
}


static kal_uint32 ctp_gslX680_command(kal_uint32 cmd, void* p1, void* p2) // p1: input p2: output
{
	return 0;
}

/*
This structure is to initialize function pointer to CTP driver.
NOT all function MUST BE implemented in this struct,
JUST doing function declaration is OK!!

But the following TWO functions MUST BE implemented:
ctp_focalteck_GSL_init
ctp_focalteck_GSL_get_data

Other functions should be implemented by customer for better performance.
*/
CTP_customize_function_struct ctp_custom_func=
{
	ctp_gslX680_init,
	ctp_gslX680_device_mode,
	ctp_gslX680_hisr,
	ctp_gslX680_get_data,
	ctp_gslX680_parameters,
	ctp_gslX680_power_on,
	ctp_gslX680_command
};

/*
Upper layer use this hook to get CTP driver function
*/
CTP_customize_function_struct *ctp_GetFunc(void)
{
	return(&ctp_custom_func);
}
#endif //#if defined(__TOUCH_PANEL_CAPACITY__)
