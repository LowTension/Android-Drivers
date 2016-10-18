/*
 *
 * GMACommon.h 2014-06-04
 *
 */
#ifndef GMACOMMON_H
#define GMACOMMON_H

#include <stdio.h>     //frpintf
#include <stdlib.h>    //atoi
#include <string.h>    //memset
#include <unistd.h>
#include <stdarg.h>    //va_list
#include <utils/Log.h> //LOGV
#include <errno.h>     //errno

/*** Constant definition ******************************************************/
#undef LOG_TAG
#define LOG_TAG "GMA_Daemon"

#define DBG_LEVEL0	0	// Critical
#define DBG_LEVEL1	1	// Notice
#define DBG_LEVEL2	2	// Information
#define DBG_LEVEL3	3	// Debug
#define DBG_LEVEL4	4	// Verbose

#ifndef DBG_LEVEL
#define DBG_LEVEL	DBG_LEVEL0
#endif

#define DATA_AREA01	0x0001
#define DATA_AREA02	0x0002
#define DATA_AREA03	0x0004
#define DATA_AREA04	0x0008
#define DATA_AREA05	0x0010
#define DATA_AREA06	0x0020
#define DATA_AREA07	0x0040
#define DATA_AREA08	0x0080
#define DATA_AREA09	0x0100
#define DATA_AREA10	0x0200
#define DATA_AREA11	0x0400
#define DATA_AREA12	0x0800
#define DATA_AREA13	0x1000
#define DATA_AREA14	0x2000
#define DATA_AREA15	0x4000
#define DATA_AREA16	0x8000

/* Debug area definition */
#define GMADATA_DR			DATA_AREA01
#define GMADATA_TM			DATA_AREA02
#define GMADATA_PE			DATA_AREA03
#define GMADATA_ME			DATA_AREA04
#define GMADATA_MO			DATA_AREA05
#define GMADATA_PO			DATA_AREA07
#define GMADATA_HLPS		DATA_AREA08
#define GMADATA_INIT		DATA_AREA09	/*<! Initial parameter */
#define GMADATA_VEC			DATA_AREA10
#define GMADATA_FST			DATA_AREA11
#define GMADATA_MAGDRV		DATA_AREA12	/*<! Driver data */


#ifndef ENABLE_GMADEBUG
#define ENABLE_GMADEBUG		0	/* Eanble debug output when it is 1. */
#endif

#ifndef OUTPUT_STDOUT
#define OUTPUT_STDOUT		0	/* Output to stdout when it is 1. */
#endif

#define OPMODE_CONSOLE		0x01
//#define OPMODE_FST			0x02

/***** Debug output ******************************************/
#if ENABLE_GMADEBUG
#if OUTPUT_STDOUT
#define GMADEBUG(level, format, ...) \
    (((level) <= DBG_LEVEL) \
	  ? (fprintf(stdout, (format), ##__VA_ARGS__)) \
	  : ((void)0))
#else
#define GMADEBUG(level, format, ...) \
	ALOGD_IF(((level) <= DBG_LEVEL), (format), ##__VA_ARGS__)
#endif
#else
#define GMADEBUG(level, format, ...)
#endif

/***** Dbg Area Output ***************************************/
#if ENABLE_GMADEBUG
#define GMADATA(flag, format, ...)  \
	((((int)flag) & g_dbgzone) \
	  ? (fprintf(stdout, (format), ##__VA_ARGS__)) \
	  : ((void)0))
#else
#define GMADATA(flag, format, ...)
#endif

/***** Data output *******************************************/
#if OUTPUT_STDOUT
#define GMADUMP(format, ...) \
	fprintf(stderr, (format), ##__VA_ARGS__)
#else
#define GMADUMP(format, ...) \
	ALOGD((format), ##__VA_ARGS__)
#endif


/***** Error output *******************************************/
#define GMAERROR \
	((g_opmode == 0) \
	  ? (ALOGE("%s:%d Error.", __FUNCTION__, __LINE__)) \
	  : (fprintf(stderr, "%s:%d Error.\n", __FUNCTION__, __LINE__)))

#define GMAERROR_STR(api) \
	((g_opmode == 0) \
	  ? (2LOGE("%s:%d %s Error (%s).", \
	  		  __FUNCTION__, __LINE__, (api), strerror(errno))) \
	  : (fprintf(stderr, "%s:%d %s Error (%s).\n", \
	  		  __FUNCTION__, __LINE__, (api), strerror(errno))))

/*** Type declaration *********************************************************/
#define SENSOR_DATA_SIZE 		3
#define AVG_NUM 				64      /* for calibration */
/* ABS axes parameter range [um/s^2] (for input event) */

typedef union {
	struct {
		int	x;
		int	y;
		int	z;
	} u;
	int	v[SENSOR_DATA_SIZE];
} raw_data;
/*** Global variables *********************************************************/
extern int g_stopRequest;	/*!< 0:Not stop,  1:Stop */
extern int g_opmode;		/*!< 0:Daemon mode, 1:Console mode. */
extern int g_dbgzone;		/*!< Debug zone. */
#define ACC_SENSOR_GMA301		"/dev/gma301"
#define ACC_SENSOR_GMA302		"/dev/gma302"
#define ACC_SENSOR_GMA303		"/dev/gma303"
#define ACC_SENSOR_GMA305		"/dev/gma305"
#define DEVICE_SENSOR_RK		"/dev/mma8452_daemon"
#define DEVICE_SENSOR_MTK		"/dev/gsensor"


#define GlobalMems	301	/*	support official Driver IOCTL	*/
#define RockChip	400	/*	support RockChip Driver IOCTL	*/
#define AllWinner	500	/*	support AllWinner Driver IOCTL	*/
#define MTK			600	/*	support mtk driver				*/
/*** Prototype of function ****************************************************/

#define ABS(a) ((a) < 0 ? -(a) : (a))
/**
* support RK driver
* GSENSOR_IO	0xA1
*/
#define GSENSOR_IO	0xA1
#define GBUFF_SIZE				12	/* Rx buffer size */

/**
* support RK driver
* GSENSOR_IOCTL_MAGIC			'a'
*/
#define GSENSOR_IOCTL_MAGIC			'a'


//=======for MTK=================尚需平台配合驅動程式來驗證
#define DEFAULT_SENSITIVITY 	128 	/* raw data sensitivity */
#define	GRAVITY_EARTH_1000		9807	/*	1G = 9807*/
#define ABSMAX					(GRAVITY_EARTH_1000 * 6)
#define ABSMIN					(-GRAVITY_EARTH_1000 * 6)

#endif 



