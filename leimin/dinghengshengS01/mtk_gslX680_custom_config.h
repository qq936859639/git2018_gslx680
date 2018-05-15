#include "tpd.h"
#include <linux/stddef.h>
//#include "tpd_custom_fts.h"
//#include "cust_gpio_usage.h"
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <asm/unistd.h>
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mt_boot.h>
#include <mach/irqs.h>
//#include <cust_eint.h>
#include <linux/jiffies.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#define TPD_POWER_SOURCE_CUSTOM         	PMIC_APP_CAP_TOUCH_VDD


#ifdef GSLX680_CONFIG_DHD_X09_HSD_HST_YS_GES
#include "gslX680_dhd_x09_hesd_hest_yusheng_gesture.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON
#define GSL_IDENTY_TP

#elif defined(GSLX680_CONFIG_DHD_X09_COMM)
#include "960_540_mtk_gslX680_add_gesuter.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON
#define GSL_IDENTY_TP

#elif defined(TPCONFIG_DHD_X10_GSL1688)
//#error xxxx_TPCONFIG_DHD_X10_GSL1688_xxxxxxx
#include "gslX680_dhd_x10_heshengda.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#elif defined(TPCONFIG_DHD_X12_GSL1688_HESHENGTAI)
//#error xxxx_TPCONFIG_DHD_X10_GSL1688_xxxxxxx
#include "gslX680_dhd_x12_heshengtai.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#elif defined(TPCONFIG_DHD_X10_HSD_YS_GSL1688)
//#error xxxx_TPCONFIG_DHD_X10_HSD_YS_GSL1688_xxxxxxx
#include "mtk_gslX680_x10_hsd_ys_cob.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON
#define TPBUTTON_BACK100_HOME250_MENU400_SEARCH470_W60_H50_FW

#elif defined(TPCONFIG_DHD_H02_FW_GSL1688_COMM)
//#error xxxx_TPCONFIG_DHD_X10_GSL1688_xxxxxxx
#include "gslX680_dhd_m10_h02fw_yusheng.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#elif defined(GSLX680_CONFIG_DHD_H02)
//#error xxxx_TPCONFIG_DHD_X10_GSL1688_xxxxxxx
#include "gslX680_dhd_h02_qhd_yusheng.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#elif defined(GSLX680_DHD_H02_QHD_20150525)
#include "gslX680_dhd_h02_qhd_20150525.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON
#define GSL1680_EXCHANGE_X_Y
#define GSL_MONITOR 
#elif defined(GSLX680_JHM_M058_HD_20170321)
#include "gslX680_jhm_m058_siliwei.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON

#elif defined(GSL1691_DHS_S01_540X1080_CHENHE)
//20170720
#include "gsl1691_dhs_s01_540x1080_chenhe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#elif defined(GSL1691_DHS_S01_540X1080_CHENHE_SCALE_TO_LCM)
//20170720
#include "gsl1691_dhs_s01_540x1080_chenhe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#define GSL_SCALE_MAX_WIDTH 540
#define GSL_SCALE_MAX_HIGHT 1080

#elif defined(GSL1691_DHS_S01_480X960_CHENHE)
//20170720
#include "gsl1691_dhs_s01_480x960_chenhe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#elif defined(GSL1691_DHS_S01_720X1440_CHENHE)
//20170720
#include "gsl1691_dhs_s01_720x1440_chenhe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

//#define GSL_SCALE_MAX_WIDTH 720
//#define GSL_SCALE_MAX_HIGHT 1440

#elif defined(GSLX680_QHD_ZYDK_V1_SHANGKE)
#include "gslX680_zydk_qhd_v1_shangke.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON
#define TPBUTTON_BACK100_HOME250_MENU400_SEARCH470_W60_H50_Y2048

#elif defined(GSL1681E_NIUJIE_V105_HD_SKE444)
#include "gsl1681e_niujie_v105_hd_ske444.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON
#define TPBUTTON_BACK90_HOME270_MENU450_SEARCH470_W60_H50_Y2048

#elif defined(GSL1691_DHS_S02_720X1440_CHENHE)
//20171215
#include "gsl1691_dhs_s02_720x1440_chenhe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON

#elif defined(GSL1691_DHS_S02_720X1440_CHENHE_SCALE_TO_LCM)
//20180120
#include "gsl1691_dhs_s02_720x1440_chenhe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON
#define GSL_SCALE_MAX_WIDTH 720
#define GSL_SCALE_MAX_HIGHT 1440

#elif defined(GSLX680_DHD_F18_HESHENGDA_CHENGHE_SCALE_TO_LCM)
#include "gslX680_dhd_f18_chenghe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON

#define GSL_SCALE_MAX_WIDTH 720
#define GSL_SCALE_MAX_HIGHT 1440
#define GSL_IDENTY_TP_BY_DAC
#define GSL_NOID_VERSION
#elif defined(GSL1691_DHS_S01_480X960_CHENHE_HESHENGDA)
#include "gsl1691_dhs_s01_480x960_heshengda_chenghe.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON

#define GSL_SCALE_MAX_WIDTH 720
#define GSL_SCALE_MAX_HIGHT 960
#define GSL_IDENTY_TP_BY_DAC
#define GSL_NOID_VERSION
#elif defined(GSLX680_DHD_Y01_QHD_CHENHE_HESHENGDA)
#include "gslx680_dhd_y01_qhd_chenhe_heshengda.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON
#define GSL_IDENTY_TP
#elif defined(GSLX680_DHD_E01_QHD_CHENHE_HESHENGDA)
#include "gslX680_dhd_e01_qhd_chenhe_heshengda.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON
#define GSL_IDENTY_TP_BY_DAC
#elif defined(GSLX680_DHD_Y02_QHD_CHENHE_HESHENGDA)
#include "gslX680_dhd_y02_qhd_chenhe_heshengda.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON
#define GSL_IDENTY_TP
#elif defined(GSLX680_DHD_Y03_QHD_YUECHENG)
#include "gslX680_dhd_y03_qhd_yuecheng.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON
#define TPBUTTON_BACK100_HOME250_MENU400_SEARCH470_W60_H50_Y2048
#define GSL_IDENTY_TP
#define GSL_SCALE_MAX_WIDTH 540
#define GSL_SCALE_MAX_HIGHT 960

#elif defined(GSL1691_DHS_P02_480X960_HESHENGDA)
//20180323
#include "gsl1691_dhs_p02_480x960_heshengda.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE 
#define TPD_HAVE_BUTTON
#define TPBUTTON_BACK100_HOME250_MENU400_SEARCH470_W60_H50_Y2048

#else
#include "gslX680_dhd_x10_heshengda.h"
#define TPD_PS_SUPPORT
#define GSL_GESTURE
#define TPD_HAVE_BUTTON
#endif




#ifdef GSL_GESTURE

/*
		gesture feature config part
*/

//use tpd-button, should open TPD_HAVE_BUTTON

#if defined(TPCONFIG_GESTURE_DEF_ON)||defined(MAINDEF_TP_GESTURE_ALLON)
//#define CUST_KEY_GES_LEFT		1,KEY_GES_LEFT
//#define CUST_KEY_GES_RIGHT		1,KEY_GES_RIGHT
//#define CUST_KEY_GES_UP		    	1,KEY_GES_UP
//#define CUST_KEY_GES_DOWN		1,KEY_GES_DOWN
#define CUST_KEY_GES_DOUBLECLICK	1,KEY_GES_DCLICK
#define CUST_KEY_GES_O		    1,KEY_GES_O
#define CUST_KEY_GES_W		    1,KEY_GES_W
#define CUST_KEY_GES_M		    1,KEY_GES_M
#define CUST_KEY_GES_E		    1,KEY_GES_E
#define CUST_KEY_GES_C		    1,KEY_GES_C
//#define CUST_KEY_GES_S		    1,KEY_GES_S
//#define CUST_KEY_GES_V		    1,KEY_GES_V
//#define CUST_KEY_GES_Z		    1,KEY_GES_Z

#define GESTURE_FEATURE_DEF_ONOFF 1

#elif defined(MAINDEF_TP_GESTURE_ALLON_WITHOUT_DCLK)
//#define CUST_KEY_GES_LEFT             1,KEY_GES_LEFT
//#define CUST_KEY_GES_RIGHT            1,KEY_GES_RIGHT
//#define CUST_KEY_GES_UP                       1,KEY_GES_UP
//#define CUST_KEY_GES_DOWN             1,KEY_GES_DOWN
#define CUST_KEY_GES_DOUBLECLICK        0,KEY_GES_DCLICK
#define CUST_KEY_GES_O              1,KEY_GES_O
#define CUST_KEY_GES_W              1,KEY_GES_W
#define CUST_KEY_GES_M              1,KEY_GES_M
#define CUST_KEY_GES_E              1,KEY_GES_E
#define CUST_KEY_GES_C              1,KEY_GES_C
//#define CUST_KEY_GES_S                    1,KEY_GES_S
//#define CUST_KEY_GES_V                    1,KEY_GES_V
//#define CUST_KEY_GES_Z                    1,KEY_GES_Z

#define GESTURE_FEATURE_DEF_ONOFF 1

#else //GSLX680_CONFIG_DHD_H02
//#define CUST_KEY_GES_LEFT		1,KEY_GES_LEFT
//#define CUST_KEY_GES_RIGHT		1,KEY_GES_RIGHT
//#define CUST_KEY_GES_UP		    	1,KEY_GES_UP
//#define CUST_KEY_GES_DOWN		1,KEY_GES_DOWN
#define CUST_KEY_GES_DOUBLECLICK	1,KEY_GES_DCLICK
#define CUST_KEY_GES_O		    1,KEY_GES_O
#define CUST_KEY_GES_W		    1,KEY_GES_W
#define CUST_KEY_GES_M		    1,KEY_GES_M
#define CUST_KEY_GES_E		    1,KEY_GES_E
#define CUST_KEY_GES_C		    1,KEY_GES_C
//#define CUST_KEY_GES_S		    1,KEY_GES_S
//#define CUST_KEY_GES_V		    1,KEY_GES_V
//#define CUST_KEY_GES_Z		    1,KEY_GES_Z

#define GESTURE_FEATURE_DEF_ONOFF 0
#endif

#endif//GSL_GESTURE




#ifdef TPD_HAVE_BUTTON

/*
		keypad feature config part
*/
//
#if defined(TPBUTTON_BACK100_HOME250_MENU400_SEARCH470_W60_H50_FW)
#define TPD_KEY_COUNT	4
#define TPD_KEYS		{KEY_BACK, KEY_HOMEPAGE, KEY_MENU, KEY_SEARCH}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{100, 900, 60, 50},{250, 900, 60, 50},{400,900, 60, 50},{470, 2048, 60, 50}}

#elif defined(TPBUTTON_BACK100_HOME250_MENU400_SEARCH470_W60_H50_Y2048)
#define TPD_KEY_COUNT	4
#define TPD_KEYS		{KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{100, 2048, 60, 50},{250, 2048, 60, 50},{400,2048, 60, 50},{470, 2048, 60, 50}}


//common define
#elif defined(TPBUTTON_BACK90_HOME270_MENU450_SEARCH470_W60_H50_Y2048)
#define TPD_KEY_COUNT	4
#define TPD_KEYS		{KEY_BACK, KEY_HOMEPAGE,KEY_MENU , KEY_SEARCH}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{90, 2048, 60, 50},{270, 2048, 60, 50},{450,2048, 60, 50},{600, 2048, 60, 50}}

#else
#define TPD_KEY_COUNT	4
#define TPD_KEYS		{KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{100, 1300, 60, 50},{250, 1300, 60, 50},{400,1300, 60, 50},{470, 2048, 60, 50}}
#endif

#endif//TPD_HAVE_BUTTON


