#ifndef __GSL_TS_DRIVER_H__
#define __GSL_TS_DRIVER_H__
/*********************************/
#define TPD_HAVE_BUTTON		//按键宏
#define GSL_ALG_ID			//有没有id算法
#define GSL_COMPATIBLE_CHIP	//芯片兼容宏
#define GSL_THREAD_EINT		//线程中断
#define GSL_DEBUG			//调试
#define TPD_PROC_DEBUG		//adb调试
#define GSL_TIMER			//定时器宏


//#define GSL_IRQ_CHECK

#if defined (BIRD_TPD_PROXIMITY)
#define TPD_PROXIMITY		//距离传感器宏
#else
//#define TPD_PROXIMITY		//距离传感器宏
#endif


#define GSL_GESTURE      //手势唤醒宏

//#define GSL_GPIO_IDT_TP	//GPIO兼容	
#if defined(MX2116_WPF_D5015)
#define GSL_DRV_WIRE_IDT_TP	//驱动线兼容
#endif

//#define GSL9XX_VDDIO_1800 1   //9系列IC宏

#if ( defined(MX2116_SMT_FW) ||defined(MX2103_SMT_FW)|| defined(MX2116_ARES_A5086_FW)|| defined(MX2116_ARES_A5086_QHD) || defined(MX2116_WPF_D5015)|| defined(MX2116_WPF_D5015_TEST) || defined(MX2116_BER_S4513) || defined(MX2116_BER_S5075) ||defined(MX2116_WPF_D5012)||defined(MX2116_NF_S5051)|| defined(MX2103_JC_C09_FWVGA)|| defined(MX2116_WPF_D5019)||defined (MX2116_ARES_A5083B_QHD) ||defined(MX2109F_ARES_A5085)||defined(MX2116G_BER_S5088_FW)||defined(MX2116G_WPF_D5023_HD720))
#define TPD_POWER_VTP28_USE_VCAMA   //[add for mx2116 2015-11-03]
#endif

#define GSL_PAGE_REG    0xf0
#define GSL_CLOCK_REG   0xe4
#define GSL_START_REG   0xe0
#define POWE_FAIL_REG   0xbc
#define TOUCH_INFO_REG  0x80
#define TPD_DEBUG_TIME	0x20130424
struct gsl_touch_info
{
	int x[10];
	int y[10];
	int id[10];
	int finger_num;	
};

struct gsl_ts_data {
	struct i2c_client *client;
	struct workqueue_struct *wq;
	struct work_struct work;
	unsigned int irq;
	//struct early_suspend pm;
};

/*button*/
#if defined (MX1092_ARES_5091)
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM            {{25,1707,20,20},{230,1707,20,20},{450,1707,20,20}}
fffff<>iiii
#elif defined(MX1092_HYF_V7)
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM            {{25,1707,20,20},{230,1707,20,20},{450,1707,20,20}}
 dddddd<>iiiiiiiii   recent
#else
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_RECENT,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM            {{25,1707,20,20},{230,1707,20,20},{450,1707,20,20}}
//sssss<>iiiiii
#endif


#ifdef GSL_ALG_ID 
extern unsigned int gsl_mask_tiaoping(void);
extern unsigned int gsl_version_id(void);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern void gsl_DataInit(int *ret);


#endif
/* Fixme mem Alig */
struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};
#ifdef GSL_DRV_WIRE_IDT_TP
#include "gsl_idt_tp.h"
#endif

#if defined (MX1092_ARES_5091)
#include "gsl_ts_mx1092_ares_5091_fw.h"
#elif defined (MX1092_HYF_V7)
#include "gsl_ts_mx1092_hyf_v7_fw.h"
#elif defined (VH2521_A5590)
#include "gsl_ts_a5590.h"
#elif defined (MX1092_HYF_A1_FWVGA)
#include "gsl_ts_fw_hyf_a1.h"
#elif defined (MX1092_JY_A5006)
#include "gsl_ts_mx1092_jy_a5006.h"
#elif defined (MX1091_HYF_9300)
#include "gsl_ts_mx1091_hyf_9300_fw.h"
#elif defined (MX1093_GL4510)
#include "gsl_ts_mx1093_gl4510_fw.h"
#elif defined(MX1091_HYF_A1_PRO)
#include "gsl_ts_mx1091_hyf_a1_pro.h"
#elif defined (MX1091_ARES_A5088)
#include "gsl_ts_mx1091_ares_a5088.h"
#elif defined (MX1091_ARES_A5088_FW)
#include "gsl_ts_mx1091_ares_a5088_fw.h"
#elif defined (MX1091_ARES_A5019)
#include "gsl_ts_mx1091_ares_a5019.h"
#elif defined (MX1091_ARES_A5019_FW)
#include "gsl_ts_mx1091_ares_a5019_fw.h"
#elif defined(MX1091_ARES_A5502)
#include "gsl_ts_mx1091_ares_a5502.h"
#elif defined (VH2510_WPF_D5013_FW)
#include "gsl_ts_vh2510_wpf_d5013_fw.h"
#elif defined (VH2510_WPF_D5016)
#include "gsl_ts_vh2510_wpf_d5016.h"
#elif defined (VH2510_NUOFEI_S5042B)
#include "gsl_ts_vh2510_wpf_s5042b.h"
#elif defined (MX2116_SMT_FW)
#include "gsl_ts_mx2116_smt_fw.h"
#elif defined (MX1098_SMT_FW)
#include "gsl_ts_mx2116_smt_fw.h"
#elif defined (MX2103_SMT_FW)
#include "gsl_ts_mx2116_smt_fw.h"
#elif defined (MX2116_ARES_A5086_FW)
#include "gsl_ts_mx2116_ares_a5086_fw.h"
#elif defined (MX2116_ARES_A5086_QHD)
#include "gsl_ts_mx2116_ares_a5086_qhd.h"
#elif defined (MX2116_WPF_D5015)
//#include "gsl_ts_mx2116_wpf_d5015_fw_mwd.h"
//#include "gsl_ts_mx2116_wpf_d5015_fw_hsd.h"
//#include "gsl_ts_mx2116_wpf_d5015_fw_mwd_gsl_1691.h"
#include "gsl_ts_mx2116_wpf_d5015_fw_mwd_gsl1691_hsd_gsl1691.h"
#elif defined (MX2116_WPF_D5015_TEST)
#include "gsl_ts_mx2116_wpf_d5015_fw_mwd.h"
#elif defined (MX2116_WPF_D5019)
#include "gsl_ts_mx2116_wpf_d5019_fw.h" 
#elif defined (MX2116_BER_S4513)
#include "gsl_ts_mx2116_ber_s4513_fw.h"
#elif defined (MX2116_BER_S4513_QHD)
#include "gsl_ts_mx2116_ber_s4513_qhd.h"
#elif defined (MX2116_BER_S5075)
#include "gsl_ts_mx2116_ber_s5075_fw.h"
#elif defined (MX2116_WPF_D5012)
#include "gsl_ts_mx2116_wpf_d5012.h"
#elif defined (MX2116_WPF_D4513)
#include "gsl_ts_mx2116_wpf_d4513_fw.h"
#elif defined (MX2116_WPF_D4513_QHD)
#include "gsl_ts_mx2116_wpf_d4513_qhd.h"
#elif defined (MX2116_FD_VE4518)
#include "gsl_ts_mx2116_fd_ve4518_fw.h"
#elif defined (MX2116_BER_S5513)
#include "gsl_ts_mx2116_ber_s5513_fw.h"
#elif defined (MX2109F_ARES_5503)
#include "gsl_ts_mx2109f_ares_5503_fw.h"
#elif defined (MX2109F_ARES_A5085)
#include "gsl_ts_mx2109f_ares_a5085_fw.h"
#elif defined (MX2109F_ARES_A5085_QHD)
#include "gsl_ts_mx2109f_ares_a5085_qhd.h"
#elif defined (MX2116_NF_S5051)
#include "gsl_ts_mx2116_nf_s5051.h"
#elif defined (MX2116_NF_S5063_CTA)
#include "gsl_ts_mx2116_nf_s5063_fw.h"
#elif defined (MX2116_NF_S5559)
#include "gsl_ts_mx2116_nf_s5559_fw.h"
#elif defined (MX1092_JC_C03_FWVGA)
#include "gsl_ts_mx1092_jc_c03_fw.h"
#elif defined (MX2103_JC_C09_FWVGA)
#include "gsl_ts_mx2103_jc_c09_fw.h"
#elif defined (MX2116_NF_S5062)
#include "gsl_ts_mx2116_nf_s5062.h"
#elif defined (MX2109F_HDS_X1508)
#include "gsl_ts_mx2109f_hds_x1508_fw.h"
#elif defined (MX2111_WPF_D5020)
#include "gsl_ts_mx2111_wpf_d5020_hd720.h"
#elif defined (MX2116_NF_S5062_QHD)
#include "gsl_ts_mx2116_nf_s5062_qhd.h"
#elif defined (MX2116_JC_C11)
#include "gsl_ts_mx2116_jc_c11.h"
#elif defined (MX2116_JC_C12)
#include "gsl_ts_mx2116_jc_c12.h"
#elif defined (MX2109F_WPF_D5018)
#include "gsl_ts_mx2109f_wpf_d5018_fw.h"
#elif defined (MX2116_XHT_771)
#include "gsl_ts_mx2116_xht_771_fw.h"
#elif defined (MX2116_BER_S6001)
#include "gsl_ts_mx2116_ber_s6001_hd720_fw.h"
#elif defined (MX2116_HYF_JD3)
#include "gsl_ts_mx2116_hyf_jd3_fw.h"
#elif defined (MX2116_ARES_A5083B_QHD)
#include "gsl_ts_mx2116_ares_a5083b_qhd.h"
#elif defined (MX2116G_BER_S5088_FW)
#include "gsl_ts_mx2116g_ber_s5088_fw_hsd_gsl1691.h"
#elif defined (MX2116G_WPF_D5023_HD720)
#include "gsl_ts_mx2116g_wpf_d5023_hd720_hsd_gsl1691.h"
#else
#include "gsl_ts_fw.h"
#endif

static unsigned char gsl_cfg_index = 0;

struct fw_config_type
{
	const struct fw_data *fw;
	unsigned int fw_size;
	unsigned int *data_id;
	unsigned int data_size;
};
static struct fw_config_type gsl_cfg_table[9] = {
#if defined( MX2116_WPF_D5015)
/*1*/{GSLX680_FW_HSD,(sizeof(GSLX680_FW_HSD)/sizeof(struct fw_data)),gsl_config_data_id_hsd,(sizeof(gsl_config_data_id_hsd)/4)},
/*2*/{GSLX680_FW_MWD,(sizeof(GSLX680_FW_MWD)/sizeof(struct fw_data)),gsl_config_data_id_mwd,(sizeof(gsl_config_data_id_mwd)/4)},
#else
/*0*/{GSLX680_FW,(sizeof(GSLX680_FW)/sizeof(struct fw_data)),gsl_config_data_id,(sizeof(gsl_config_data_id)/4)},
#endif
};

#endif
