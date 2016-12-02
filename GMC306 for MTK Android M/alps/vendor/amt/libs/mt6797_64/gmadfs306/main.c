/******************************************************************************
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
#include "GMFS_Common.h"
#include "GMFS_Compass.h"
#include "GMFS_Disp.h"
#include "GMFS_FileIO.h"
#include "GMFS_Measure.h"
#include "GMFS_APIs.h"
#include "libGME_OSS/gMfusion.h"

#ifndef WIN32
#include <sched.h>
#include <pthread.h>
#include <linux/input.h>
#include <sys/stat.h>    //chmod
#endif
#define CONVERT_M			6
#define CONVERT_M_DIV		100			// 6/100 = CONVERT_M
#define CONVERT_O			1
#define CONVERT_O_DIV		64			// 1/64 = CONVERT_O

#define CONVERT_Q16			1
#define CONVERT_Q16_DIV		65536	
#define CONVERT_Q14_DIV		16384	

/*** Constant definition ******************************************************/
#define ERROR_INITDEVICE		(-1)
#define ERROR_OPTPARSE			(-2)
#define ERROR_SELF_TEST			(-3)
#define ERROR_READ_FUSE			(-4)
#define ERROR_INIT				(-5)
#define ERROR_GETOPEN_STAT		(-6)
#define ERROR_STARTCLONE		(-7)
#define ERROR_GETCLOSE_STAT		(-8)

#define GME_SELFTEST_MIN_X	-100
#define GME_SELFTEST_MAX_X	100
#define GME_SELFTEST_MIN_Y	-100
#define GME_SELFTEST_MAX_Y	100
#define GME_SELFTEST_MIN_Z	-1000
#define GME_SELFTEST_MAX_Z	-300

#define CONVERT_ACC(a)	((int)((a) * 720 / 9.8f))

#define CONVERT_MAG(m)	((int)((m) * 1000))

//#define CONVERT_MAG(m)	((int)((m) / 0.06f))

#define CONVERT_ORI(o)	((int)((o) * 64))

/*** Global variables *********************************************************/
int g_stopRequest = 0;
int g_opmode = 0;
int g_dbgzone = 0;
int g_mainQuit = GME_D_FALSE;

/* Static variable. */
static pthread_t s_thread;  /*!< Thread handle */

/*** Sub Function *************************************************************/
/*!
  Read sensitivity adjustment data from fuse ROM.
  @return If data are read successfully, the return value is #GME_SUCCESS.
   Otherwise the return value is #GME_ERROR.
  @param[out] regs The read ASA values. When this function succeeds, ASAX value
   is saved in regs[0], ASAY is saved in regs[1], ASAZ is saved in regs[2].
 */
int16 GMFS_ReadConf(
		uint8	regs[3]
)
{
	BYTE conf[GME_SENSOR_CONF_SIZE];

#ifdef GME_VALUE_CHECK
	if (GME_SENSOR_CONF_SIZE != 3) {
		GMEERROR_STR("You may refer invalid header file.");
		return GME_ERROR;
	}
#endif

	if (GME_D_GetSensorConf(conf) != GME_D_SUCCESS) {
		GMEERROR;                                          
		return GME_ERROR;
	}
	regs[0] = conf[0];
	regs[1] = conf[1];
	regs[2] = conf[2];

	GMEDEBUG(GMEDATA_DUMP, "%s: asa(dec)=%d,%d,%d\n",
			__FUNCTION__, regs[0], regs[1], regs[2]);

	return GME_SUCCESS;
}

/*!
  This function calculate the duration of sleep for maintaining
   the loop keep the period.
  This function calculates "minimum - (end - start)".
  @return The result of above equation in nanosecond.
  @param end The time of after execution.
  @param start The time of before execution.
  @param minimum Loop period of each execution.
 */
struct timespec GMFS_CalcSleep(
	const struct timespec* end,
	const struct timespec* start,
	const int64_t minimum
)
{
	unsigned long long int endL;
	unsigned long long int startL;
	unsigned long long int diff;

	struct timespec ret;

	endL=(unsigned long long int)end->tv_sec;
	endL*=1000000000;
	endL+=(unsigned long long int)end->tv_nsec;
	
	startL=(unsigned long long int)start->tv_sec;
	startL*=1000000000;
	startL+=(unsigned long long int)start->tv_nsec;
	diff = (unsigned long long int)minimum;

	diff -= (endL - startL);

 	ret.tv_sec = 0;
	ret.tv_nsec = diff % minimum;
	return ret;
}

/*!
  Get interval of each sensors from device driver.
  @return If this function succeeds, the return value is #GME_SUCCESS.
   Otherwise the return value is #GME_ERROR.
  @param flag This variable indicates what sensor frequency is updated.
  @param minimum This value show the minimum loop period in all sensors.
 */
int16 GMFS_GetInterval(
		uint16*  flag,
		int64_t* minimum
)
{
	/* Accelerometer, Magnetometer, Fusion */
	/* Delay is in nano second unit. */
	/* Negative value means the sensor is disabled.*/
	int64_t delay[GME_NUM_SENSORS];
	int i;

#ifdef GME_VALUE_CHECK
	if (GME_NUM_SENSORS != 3) {
		GMEERROR_STR("You may refer invalid header file.");
		return GME_ERROR;
	}
#endif

	if (GME_D_GetDelay(delay) != GME_D_SUCCESS) {
		GMEERROR;
		return GME_ERROR;
	}
	GMEDEBUG(GMEDATA_LOOP, "delay[A,M,O]=%lld,%lld,%lld\n",
		delay[0], delay[1], delay[2]);

	/* update */
	*minimum = 1000000000;
	*flag = 0;
	for (i=0; i<GME_NUM_SENSORS; i++) {
		/* Set flag */
		if (delay[i] > 0) {
			*flag |= 1 << i;
			if (*minimum > delay[i]) {
				*minimum = delay[i];
			}
		}
	}
	return GME_SUCCESS;
}

/*!
  If this program run as console mode, measurement result will be displayed
   on console terminal.
  @return None.
 */
void GMFS_OutputResult( 
	const	uint16			flag,
	const	GMSENSOR_DATA*	acc,
	const	GMSENSOR_DATA*	mag,
	const	GMSENSOR_DATA*	ori
)
{
	int buf[GME_YPR_DATA_SIZE];     // 
	static int mag_status_count=0;
#ifdef GME_VALUE_CHECK
	if (GME_YPR_DATA_SIZE < 12) {
		GMEERROR_STR("You may refer invalid header file.");
		return;
	}
#endif
	/* Store to buffer */
	memset(buf,0,sizeof(buf));
	buf[0] = flag;					/* Data flag */
	buf[1] = CONVERT_ACC(acc->x);	/* Ax */
	buf[2] = CONVERT_ACC(acc->y);	/* Ay */
	buf[3] = CONVERT_ACC(acc->z);	/* Az */
	//buf[4] = acc->status;			/* Acc status */

	buf[9] = CONVERT_MAG(mag->x);	/* Mx , unit: uT*/
	buf[10] = CONVERT_MAG(mag->y);	/* My , unit: uT*/
	buf[11] = CONVERT_MAG(mag->z);	/* Mz , unit: uT*/
	if(mag_status_count<1)
	{
		mag_status_count;
		buf[4] = 2;//mag->status;			/* Mag status */
	}
	else
		buf[4] = mag->status;			/* Mag status */
	ALOGI("%s:%d,gmadfs Cali aaa=%d\n",__func__,__LINE__,mag->status);
	//buf[9]=  (int)(out[6]*CONVERT_Q16_DIV);
	//buf[10]= (int)(out[7]*CONVERT_Q16_DIV);
	//buf[11]= (int)(out[8]*CONVERT_Q16_DIV);
	//buf[12]=2;
	if(mag_status_count<1)
	{
		mag_status_count++;
		buf[8] = 2;//ori->status;			/* Ori status */
	}
	else
		buf[8] = ori->status;	/* yaw */
	buf[13] = CONVERT_ORI(ori->x);	/* yaw */
	buf[14] = CONVERT_ORI(ori->y);	/* pitch */
	buf[15] = CONVERT_ORI(ori->z);	/* roll */
                                                      
	//buf[22] = (int)(rv[0]*CONVERT_Q16_DIV);	/* x*sin(thita/2) */
	//buf[23] = (int)(rv[1]*CONVERT_Q16_DIV);	/* y*sin(thita/2) */
	//buf[24] = (int)(rv[2]*CONVERT_Q16_DIV);	/* z*sin(thita/2) */
	//buf[25] = (int)(rv[3]*CONVERT_Q16_DIV);	/* cos(thita/2) */
	
	
	if (g_opmode & OPMODE_CONSOLE) {
		/* Console mode */
		Disp_Result(buf);
	}

	/* Set result to driver */
	GME_D_SetYPR(buf);
}


/*!
 A thread function which is raised when measurement is started.
 @param[in] args This parameter is not used currently.
 */
static void* thread_main(void* args)
{
	GMEPRMS	*prms;
	BYTE    i2cData[GME_SENSOR_DATA_SIZE]; /* ST1 ~ ST2 */
	int16	mag[3];
	int16	mstat;
	int16	acc[3];
	struct	timespec tsstart= {0, 0};
	struct	timespec tsend = {0, 0};
	struct	timespec doze;
	int64_t	minimum;
	uint16	flag;
	GMSENSOR_DATA sv_acc;
	GMSENSOR_DATA sv_mag;
	GMSENSOR_DATA sv_ori;
	GMFLOAT tmpx, tmpy, tmpz;
	int16 tmp_accuracy;
	int mag_defreq=0;

	ALOGI("%s:%d thread_main...\n",__func__,__LINE__);
	prms = (GMEPRMS *)args;
	minimum = -1;
	

	/* Initialize library functions and device */
	ALOGI("%s:%d,GMFS_Start\n",__func__,__LINE__);
	if (GMFS_Start(prms, CSPEC_SETTING_FILE) != GME_SUCCESS) {
		GMEERROR;
		goto MEASURE_END;
	}
	while (g_stopRequest != GME_TRUE) {
		/* Beginning time */
		if (clock_gettime(CLOCK_MONOTONIC, &tsstart) < 0) {
			GMEERROR;
			goto MEASURE_END;
		}
		ALOGI("%s:%d g_GetInterval\n",__func__,__LINE__);
		/* Get interval */
		if (GMFS_GetInterval(&flag, &minimum) != GME_SUCCESS) {
			GMEERROR;
			goto MEASURE_END;
		}
		ALOGI("%s:%d minimum=%lld\n",__func__,__LINE__,minimum);

		if ((flag & ACC_DATA_READY) || (flag & FUSION_DATA_READY)) {
			/* Get accelerometer */

			ALOGI("%s:%d GME_D_GetAccelerationData\n",__func__,__LINE__);
			if (GME_D_GetAccelerationData(acc) != GME_D_SUCCESS) {
				GMEERROR;
				goto MEASURE_END;
			}
			//ALOGI("%s:%d GMFS_Get_ACCELEROMETER\n",__func__,__LINE__);
			/* Calculate accelerometer vector */
			ALOGI("%s:%d GMFS_Get_ACCELEROMETER acc=(%d,%d,%d),flag=0x%02x\n",__func__,__LINE__,acc[0],acc[1],acc[2],flag);
			if (GMFS_Get_ACCELEROMETER(prms, acc, 0, &tmpx, &tmpy, &tmpz, &tmp_accuracy) == GME_SUCCESS) {
				sv_acc.x = tmpx;
				sv_acc.y = tmpy;
				sv_acc.z = tmpz;
				sv_acc.status = tmp_accuracy;
	            ALOGI("%s:%d GMFS_Get_ACCELEROMETER acc=(%d,%d,%d)\n",__func__,__LINE__,acc[0],acc[1],acc[2]);
			} else {
				flag &= ~ACC_DATA_READY;
				flag &= ~FUSION_DATA_READY;
			}
		}		
		if ((flag & MAG_DATA_READY) || (flag & FUSION_DATA_READY)) {
			/* Set to measurement mode  */

			ALOGI("%s:%d GME_D_SetMode & GME_D_GetMagneticData\n",__func__,__LINE__);

			if (GME_D_SetMode(GME_MODE_SNG_MEASURE) != GME_D_SUCCESS) {
				GMEERROR;
				goto MEASURE_END;
			}

			memset(i2cData,0,GME_SENSOR_DATA_SIZE);

			usleep(9500);//GME_MEASURE_TIME_US);					
			/* Wait for DRDY and get data from device */
			if (GME_D_GetMagneticData(i2cData) != GME_D_SUCCESS) {
				GMEERROR;
			}

			//ALOGI("%s:%d,getMdataRetry=%d\n",__func__,__LINE__,getMdataRetry);

			/* raw data to x,y,z value*/
#ifdef GME_DEVICE_GMC306 // change the index 	,GME_DEVICE_GMC306 defined in Android.mk
			i2cData[1]^=i2cData[2];i2cData[2]^=i2cData[1];i2cData[1]^=i2cData[2];
			i2cData[3]^=i2cData[4];i2cData[4]^=i2cData[3];i2cData[3]^=i2cData[4];
			i2cData[5]^=i2cData[6];i2cData[6]^=i2cData[5];i2cData[5]^=i2cData[6];
			i2cData[8]<<=3;
#endif 
			memcpy(&mag,&i2cData[1],6);
			
			/* raw data */
			
			ALOGI("SOFT,i2cData=(0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x),mag=(%5d,%5d,%5d)\n",
				i2cData[1],i2cData[2],i2cData[3],i2cData[4],i2cData[5],i2cData[6],mag[0],mag[1],mag[2]);	
			
			mstat = i2cData[0] | i2cData[8];
			ALOGI("Mag sensor data overflow bit=%02x,%02x!\n", i2cData[0], i2cData[8]);
			if(((mstat)&0x09) != 0x01)//if  !=0x01, means ==0x09, Overflow
			{
				ALOGI("Mag sensor data overflow !\n");
				flag &= ~MAG_DATA_READY;
				flag &= ~FUSION_DATA_READY;
			}

			
			/* Calculate magnetic field vector */
			if (GMFS_Get_MAGNETIC_FIELD(prms, mag, mstat, &tmpx, &tmpy, &tmpz, &tmp_accuracy,mag_defreq) == GME_SUCCESS) 
			{
				ALOGI("SOFT,sv_mag=(%8.3f,%8.3f,%8.3f,%8.3f)\n",sv_mag.x,sv_mag.y,sv_mag.z,
					sqrt(sv_mag.x*sv_mag.x+sv_mag.y*sv_mag.y+sv_mag.z*sv_mag.z));
				sv_mag.x = prms->fv_hvec.u.x;
				sv_mag.y = prms->fv_hvec.u.y;
				sv_mag.z = prms->fv_hvec.u.z;

				if(	sv_mag.status ==0 && tmp_accuracy==3){
					ALOGI("status=%d,tmp=%d",sv_mag.status , tmp_accuracy);
					ALOGI("GMFS_SaveParameters.\n");
					/* Write setting files to a file */
					if (GMFS_SaveParameters(prms, CSPEC_SETTING_FILE) != GME_SUCCESS) {
						GMEERROR_STR("GMFS_SaveParameters");
					}
				}
				sv_mag.status = prms->i16_hstatus;	
			}
			else
			{
				//OGI("SOFT,GMFS_Get_MAGNETIC_FIELD
				flag &= ~MAG_DATA_READY;
				flag &= ~FUSION_DATA_READY;
			}
			ALOGI("%s:%d,minimum=%lld,mag_defreq=%d\n",__func__,__LINE__,minimum,mag_defreq);
			mag_defreq++;
			if(minimum > 1000)
				minimum /= 1000000;
			ALOGI("%s:%d,minimum=%lld,mag_defreq=%d\n",__func__,__LINE__,minimum,mag_defreq);
			if(minimum <=  10)//50Hz
			{
				if(mag_defreq==10)
					mag_defreq=0;
			}
			else
			if(minimum ==	20)//50Hz
			{
				if(mag_defreq==5)
					mag_defreq=0;
			}
			else
			if(minimum == 60 || minimum == 66)//15.15Hz
			{
				if(mag_defreq==2)
					mag_defreq=0;
			}
			else//10 Hz,8Hz,5Hz
				mag_defreq=0;

		}
		
		if (flag & FUSION_DATA_READY) {	   
			if (GMFS_Get_ORIENTATION(prms, &tmpx, &tmpy, &tmpz, &tmp_accuracy) == GME_SUCCESS) {
				//test temporary 20160127
				sv_ori.x = tmpx;
				sv_ori.y = tmpy;
				sv_ori.z = tmpz;
				sv_ori.status = tmp_accuracy;
			} else {
				flag &= ~FUSION_DATA_READY;
			}
		}
		ALOGI("%s:%d,ACC=(%8.3f,%6.3f,%6.3f),MAG=(%8.3f,%8.3f,%8.3f),Ori=(%8.3f,%8.3f,%8.3f)\n",__func__,__LINE__,\
			sv_acc.x,sv_acc.y,sv_acc.z,\
			sv_mag.x,sv_mag.y,sv_mag.z,\
			sv_ori.x,sv_ori.y,sv_ori.z);
		ALOGI("%s:%d,fv_hvec=(%8.3f,%8.3f,%8.3f)\n",__func__,__LINE__,\
			prms->fv_hvec.u.x,prms->fv_hvec.u.y,prms->fv_hvec.u.z);

		/* Output result */
		GMFS_OutputResult(flag, &sv_acc, &sv_mag, &sv_ori);
  		/* Ending time */
		if (clock_gettime(CLOCK_MONOTONIC, &tsend) < 0) {
			GMEERROR;
			goto MEASURE_END;
		}

		if(minimum>10)//if mininum ==10 ms for fastest mode , no time to wait, because mag sensor need 10 ms to get data ready.
		{
			doze = GMFS_CalcSleep(&tsend, &tsstart, minimum*1000000);
			GMEDEBUG(GMEDATA_LOOP, "minimum=%lld nsec,Sleep: %ld nsec ,Mag_cali=%d\n",minimum, doze.tv_nsec,prms->i16_hstatus);
			nanosleep(&doze, NULL); 
		}
	}

MEASURE_END:
	printf("%s:%d MEASURE_END\n",__func__,__LINE__);

		/* Set to PowerDown mode */
	if (GME_D_SetMode(GME_MODE_POWERDOWN) != GME_D_SUCCESS) {
		GMEERROR;
	}

	/* Save parameters */
	if (GMFS_Stop(prms, CSPEC_SETTING_FILE) != GME_SUCCESS) {
		GMEERROR;
	}
	return ((void*)0);
}

/*!
  Signal handler.  This should be used only in DEBUG mode.
  @param[in] sig Event
 */
static void signal_handler(int sig)
{
	if (sig == SIGINT) {
		GMEERROR;
		g_stopRequest = 1;
		g_mainQuit = GME_D_TRUE;
	}
}
 
/*!
 Starts new thread.
 @return If this function succeeds, the return value is 1. Otherwise,
 the return value is 0.
 */
static int startClone(void *mem)
{
	pthread_attr_t attr;

	pthread_attr_init(&attr);
	g_stopRequest = 0;
	ALOGI("%s:%d startClone...\n",__func__,__LINE__);
	if (pthread_create(&s_thread, &attr, thread_main, mem) == 0) {
		return 1;
	} else {
		return 0;
	}
}

/*!
 This function parse the option.
 @retval 1 Parse succeeds.
 @retval 0 Parse failed.
 @param[in] argc Argument count
 @param[in] argv Argument vector
 @param[out] layout_patno
 */
int OptParse(
	int		argc,
	char*	argv[],
	GMFS_PATNO*	layout_patno)
{
#ifdef WIN32
	/* Static */
	/*g_opmode = OPMODE_CONSOLE; */
	g_opmode = 0;
	g_dbgzone = GMEDATA_DUMP | GMEDATA_DEBUG | GMEDATA_CONSOLE;
#else
	int		opt;
	int		optVal;

	*layout_patno = PAT_INVALID;

while ((opt = getopt(argc, argv, "sm:z:")) != -1) {
		switch(opt)
		{
			case 'm':
				optVal =atoi(optarg);
				if ((PAT1 <= optVal) && (optVal <= PAT8)) {
					*layout_patno = (GMFS_PATNO)optVal;
					GMEDEBUG(GMEDATA_DEBUG, "%s: Layout=%d\n", __FUNCTION__, optVal);
				} else 
				{	
				   	switch(-optVal)
			    	{
				    	case 1:  *layout_patno=  PAT5 ; break;//-2
				    	case 4:  *layout_patno=  PAT6 ; break;//-1
				    	case 3:  *layout_patno=  PAT7 ; break;//-4
				    	case 2:  *layout_patno=  PAT8 ; break;//-3
			    	}
				}
				
				break;
			case 's':
				g_opmode |= OPMODE_CONSOLE;
				break;
	        case 'z':
	        /* If error detected, hopefully 0 is returned. */
	        	errno = 0;
	            g_dbgzone = (int)strtol(optarg, (char**)NULL, 0); 
				GMEDEBUG(GMEDATA_DEBUG, "%s: Dbg Zone=%d\n", __FUNCTION__, g_dbgzone);
	            break;
			default:
				GMEERROR_STR("Invalid argument");
				return 0;
		}
	}

	/* If layout is not specified with argument, get parameter from driver */
	if (*layout_patno == PAT_INVALID) {
		int16_t n = 0;
		if (GME_D_GetLayout(&n) == GME_D_SUCCESS) {
			if ((PAT1 <= n) && (n <= PAT8)) {
				*layout_patno = (GMFS_PATNO)n;
			}else {
			    	switch(-n)
			    	{
				    	case 1:  *layout_patno=  PAT5 ; break;
				    	case 4:  *layout_patno=  PAT6 ; break;
				    	case 3:  *layout_patno=  PAT7 ; break;
				    	case 2:  *layout_patno=  PAT8 ; break;
			    	}
			}
		}
		GMEDEBUG(GMEDATA_DEBUG, "Layout=%d\n", n);
	}
	/* Error */
	if (*layout_patno == PAT_INVALID) {
		GMEERROR_STR("No layout is specified.");
		return 0;
	}
#endif

	return 1;
}


void ConsoleMode(void *mem)
{
	/*** Console Mode *********************************************/
	while (GME_D_TRUE) {
		/* Select operation */
		switch (Menu_Main()) {
		case MODE_Measure:
			/* Reset flag */
			g_stopRequest = 0;
			/* Measurement routine */
			thread_main(mem);
			break;

		case MODE_Quit:
			return;

		default:
			GMEERROR_STR("Unknown operation mode.\n");
			break;
		}
	}
}

int main(int argc, char **argv)
{
	int		retValue = 0, dev=-1,g_dev=-1;
	GMEPRMS		prms;
	GMFS_PATNO	pat;
	uint8		regs[3];
	/* Show the version info of this software. */
	Disp_StartMessage();
	ALOGI("%s,%d,GlobalMems Daemon start...\n",__func__,__LINE__);
#if ENABLE_GMEDEBUG
	/* Register signal handler */
	signal(SIGINT, signal_handler);
#endif
	//ALOGI("%s,%d,GlobalMems Daemon start...\n",__func__,__LINE__);

	/* Open device driver */
	retValue=GME_D_InitDevice(&dev,&g_dev);
	if( retValue!= GME_D_SUCCESS) {
		retValue = ERROR_INITDEVICE;
		goto MAIN_QUIT;
	}
	ALOGI("%s:%d, g sensor dev=%d\n",__func__,__LINE__,dev);
	
	/* Parse command-line options */
	/* This function calls device driver function to get layout */
	if (OptParse(argc, argv, &pat) == 0) {
		retValue = ERROR_OPTPARSE;
		goto MAIN_QUIT;
	}

	/* Self Test */
	//if (g_opmode & OPMODE_FST){
	if (GMFS_SelfTest() != GME_SUCCESS) {
		retValue = ERROR_SELF_TEST;
		goto MAIN_QUIT;
	}
	else
	{
		ALOGI("%s:%d GMFS_SelfTest PASS\n",__func__,__LINE__);
	}
	//}

	/* OK, then start */
	if (GMFS_ReadConf(regs) != GME_SUCCESS) {
		retValue = ERROR_READ_FUSE;
		goto MAIN_QUIT;
	}

	/* Initialize library. */
	if (GMFS_Init(&prms, pat, regs) != GME_SUCCESS) {
		retValue = ERROR_INIT;
		goto MAIN_QUIT;
	}                                

	/* Start console mode  */
	if (g_opmode & OPMODE_CONSOLE) {
		ConsoleMode((void *)&prms);
		goto MAIN_QUIT;
	}            
	ALOGI("%s:%d g_mainQuit= %d\n",__func__,__LINE__,g_mainQuit);

	/*** Start Daemon ********************************************/
	while (g_mainQuit == GME_D_FALSE) {
		ALOGI("%s:%d g_mainQuit= %d\n",__func__,__LINE__,g_mainQuit);
		int st = 0;
		/* Wait until device driver is opened. */
		if (GME_D_GetOpenStatus(&st) != GME_D_SUCCESS) {
			ALOGI("%s:%d GME_D_GetOpenStatus...GME_D_FAIL.\n",__func__,__LINE__);
			retValue = ERROR_GETOPEN_STAT;
			goto MAIN_QUIT;
		}
		else			
			ALOGI("%s:%d GME_D_GetOpenStatus...GME_D_SUCCESS.\n",__func__,__LINE__);
		
		if (st == 0) {			
			ALOGI("%s:%d Suspended.\n",__func__,__LINE__);
			GMEDEBUG(GMEDATA_LOOP, "Suspended.");
		} else {
			ALOGI("%s:%d st= %d\n",__func__,__LINE__,st);
			GMEDEBUG(GMEDATA_LOOP, "Compass Opened.");
			/* Reset flag */
			g_stopRequest = 0;
			/* Start measurement thread. */
			ALOGI("%s:%d startClone...\n",__func__,__LINE__);
			if (startClone((void *)&prms) == 0) {
				retValue = ERROR_STARTCLONE;
				goto MAIN_QUIT;
			}

			/* Wait until device driver is closed. */
			ALOGI("%s:%d GetCloseStatus...\n",__func__,__LINE__);
			if (GME_D_GetCloseStatus(&st) != GME_D_SUCCESS) {
				ALOGI("%s:%d GetCloseStatus...GME_D_FAIL.\n",__func__,__LINE__);
				retValue = ERROR_GETCLOSE_STAT;
				g_mainQuit = GME_D_TRUE;
			}
			else			
			ALOGI("%s:%d GetCloseStatus...GME_D_SUCCESS.\n",__func__,__LINE__);
			/* Wait thread completion. */
			g_stopRequest = 1;
			ALOGI("%s:%d pthread_join. g_stopRequest = %d\n",__func__,__LINE__,g_stopRequest);
			pthread_join(s_thread, NULL);
			GMEDEBUG(GMEDATA_LOOP, "Compass Closed.");
		}
	}

MAIN_QUIT:
	ALOGI("%s,%d,GlobalMems Daemon quit...\n",__func__,__LINE__);
	// Release library 
	 
	GMFS_Release(&prms);
	/* Close device driver. */
	GME_D_DeinitDevice();
	/* Show the last message. */
	Disp_EndMessage(retValue);

	return retValue;
}


