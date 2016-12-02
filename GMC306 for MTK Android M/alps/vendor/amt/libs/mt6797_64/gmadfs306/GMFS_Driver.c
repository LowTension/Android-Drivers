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
#include <fcntl.h>
#include "GMFS_Common.h"
#include "GMFS_Driver.h"

#define GME_MEASURE_RETRY_NUM	5
static int s_fdDev = -1,dev_gsensor=-1;

/*!
 Open device driver.
 This function opens both device drivers of magnetic sensor and acceleration
 sensor. Additionally, some initial hardware settings are done, such as
 measurement range, built-in filter function and etc.
 @return If this function succeeds, the return value is #GME_D_SUCCESS.
 Otherwise the return value is #GME_D_ERROR.
 */
int16_t GME_D_InitDevice(int *dev,int *g_dev)
{
	ALOGI("%s:%d,open M sensor /dev/%s",__func__,__LINE__,GME_MISCDEV_NAME);
	if (s_fdDev < 0) {
		/* Open magnetic sensor's device driver. */
		if ((s_fdDev = open("/dev/" GME_MISCDEV_NAME, O_RDWR)) < 0) {
			GMEERROR;
			ALOGI("%s:%d,open M sensor /dev/%s",__func__,__LINE__,GME_MISCDEV_NAME);
			GMEDEBUG(GMEDATA_CONSOLE,"%s:%d,open M sensor /dev/%s",__func__,__LINE__,GME_MISCDEV_NAME);
			return GME_D_ERROR;
		}
		else
		{
			ALOGI("%s:%d,open M sensor /dev/%s=%d",__func__,__LINE__,GME_MISCDEV_NAME,s_fdDev);
			GMEDEBUG(GMEDATA_CONSOLE,"%s:%d,open M sensor /dev/%s=%d",__func__,__LINE__,GME_MISCDEV_NAME,s_fdDev);
			*dev= s_fdDev;
		}
	}
	*dev= s_fdDev;

	if ((dev_gsensor = open("/dev/" GMA_MISCDEV_NAME, O_RDWR)) < 0) {			
			GMEDEBUG(GMEDATA_CONSOLE,"%s:%d,open g sensor /dev/%s",__func__,__LINE__,GMA_MISCDEV_NAME);
			ALOGI("%s:%d,open g sensor /dev/%s",__func__,__LINE__,GMA_MISCDEV_NAME);
			return GME_D_ERROR;
	}
	else
	{
		ALOGI("%s:%d,open g sensor /dev/%s=%d",__func__,__LINE__,GMA_MISCDEV_NAME,dev_gsensor);
		GMEDEBUG(GMEDATA_CONSOLE,"%s:%d,open g sensor /dev/%s=%d",__func__,__LINE__,GMA_MISCDEV_NAME,dev_gsensor);
			*g_dev=dev_gsensor;
	}

	if(s_fdDev<0 )// m fail
	   return GME_D_ERROR;
	
	if(dev_gsensor<0) // g fail
	   return -2;

	return GME_D_SUCCESS;
}

/*!
 Close device driver.
 This function closes both device drivers of magnetic sensor and acceleration
 sensor.
 */
void GME_D_DeinitDevice(void)
{
	if (s_fdDev >= 0) {
		close(s_fdDev);
		s_fdDev = -1;
	}
	if (dev_gsensor >= 0) {
		close(dev_gsensor);
		dev_gsensor = -1;
	}
}

/*!
 Writes data to a register of the GME E-Compass.  When more than one byte of
 data is specified, the data is written in contiguous locations starting at an
 address specified in \a address.
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 @param[in] address Specify the address of a register in which data is to be
 written.
 @param[in] data Specify data to write or a pointer to a data array containing
 the data.  When specifying more than one byte of data, specify the starting
 address of the array.
 @param[in] numberOfBytesToWrite Specify the number of bytes that make up the
 data to write.  When a pointer to an array is specified in data, this argument
 equals the number of elements of the array.
 */
int16_t GME_D_TxData(
		const BYTE address,
		const BYTE * data,
		const uint16_t numberOfBytesToWrite)
{
	int i;
	char buf[GME_RWBUF_SIZE];

	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	if (numberOfBytesToWrite > (GME_RWBUF_SIZE-2)) {
		GMEERROR;
		return GME_D_ERROR;
	}

	buf[0] = numberOfBytesToWrite + 1;
	buf[1] = address;

	for (i = 0; i < numberOfBytesToWrite; i++) {
		buf[i + 2] = data[i];
	}
//	printf("ioctl ECS_IOCTL_WRITE\n");
	if (ioctl(s_fdDev, ECS_IOCTL_WRITE, buf) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	} else {

#if ENABLE_GMEDEBUG
		GMEDEBUG(GMEDATA_DRV, "addr(HEX)=%02x data(HEX)=", address);
		for (i = 0; i < numberOfBytesToWrite; i++) {
			GMEDEBUG(GMEDATA_DRV, " %02x", data[i]);
		}
		GMEDEBUG(GMEDATA_DRV, "\n");
#endif
		return GME_D_SUCCESS;
	}
}

/*!
 Acquires data from a register or the EEPROM of the GME E-Compass.
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 @param[in] address Specify the address of a register from which data is to be
 read.
 @param[out] data Specify a pointer to a data array which the read data are
 stored.
 @param[in] numberOfBytesToRead Specify the number of bytes that make up the
 data to read.  When a pointer to an array is specified in data, this argument
 equals the number of elements of the array.
 */
int16_t GME_D_RxData(
		const BYTE address,
		BYTE * data,
		const uint16_t numberOfBytesToRead)
{
	int i;
	char buf[GME_RWBUF_SIZE];

	memset(data, 0, numberOfBytesToRead);

	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	if (numberOfBytesToRead > (GME_RWBUF_SIZE-1)) {
		GMEERROR;
		return GME_D_ERROR;
	}

	buf[0] = numberOfBytesToRead;
	buf[1] = address;
//	printf("ioctl ECS_IOCTL_READ\n");
	if (ioctl(s_fdDev, ECS_IOCTL_READ, buf) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	} else {
		for (i = 0; i < numberOfBytesToRead; i++) {
			data[i] = buf[i + 1];
		}
#if ENABLE_GMEDEBUG
		GMEDEBUG(GMEDATA_DRV, "addr(HEX)=%02x len=%d data(HEX)=",
				address, numberOfBytesToRead);
		for (i = 0; i < numberOfBytesToRead; i++) {
			GMEDEBUG(GMEDATA_DRV, " %02x", data[i]);
		}
		GMEDEBUG(GMEDATA_DRV, "\n");
#endif
		return GME_D_SUCCESS;
	}
}

/*!
 Reset the e-compass.
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 */
int16_t GME_D_Reset(void) {
	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
//	printf("ioctl ECS_IOCTL_RESET\n");
	if (ioctl(s_fdDev, ECS_IOCTL_RESET, NULL) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
	return GME_D_SUCCESS;
}

/*!
 Get magnetic sensor information from device. This function returns WIA value.
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 @param[out] data An information data array. The size should be larger than
 #GME_SENSOR_INFO_SIZE
 */
int16_t GME_D_GetSensorInfo(BYTE data[GME_SENSOR_INFO_SIZE])
{
	memset(data, 0, GME_SENSOR_INFO_SIZE);

	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	 //unused in kernel driver gme605_compass.c
	/*if (ioctl(s_fdDev, ECS_IOCTL_GET_INFO, data) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
	*/
	return GME_D_SUCCESS;
}

/*!
 Get magnetic sensor configuration from device. This function returns ASA value.
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 @param[out] data An configuration data array. The size should be larger than
 #GME_SENSOR_CONF_SIZE
 */
int16_t GME_D_GetSensorConf(BYTE data[GME_SENSOR_CONF_SIZE])
{
	memset(data, 0, GME_SENSOR_CONF_SIZE);

	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	if (ioctl(s_fdDev, ECS_IOCTL_GET_CONF_60X, data) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
	return GME_D_SUCCESS;
}

/*!
 Acquire magnetic data from GME E-Compass. If measurement is not done, this
 function waits until measurement completion.
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 @param[out] data A magnetic data array. The size should be larger than
 #GME_SENSOR_DATA_SIZE.
 */
int16_t GME_D_GetMagneticData(BYTE data[GME_SENSOR_DATA_SIZE])
{
	int ret;
	int i;

	memset(data, 0, GME_SENSOR_DATA_SIZE);

	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}

	for (i = 0; i < GME_MEASURE_RETRY_NUM; i++) {
		ret = ioctl(s_fdDev, ECS_IOCTL_GETDATA, data);

		if (ret >= 0) {
			/* Success */
			break;
		}
		if (errno != EAGAIN) {
			GMEERROR_STR("ioctl");
			return GME_D_ERROR;
		}
		GMEDEBUG(GMEDATA_DRV, "Try Again.");
		usleep(GME_MEASURE_TIME_US);
	}

	if (i >= GME_MEASURE_RETRY_NUM) {
		GMEERROR;
		return GME_D_ERROR;
	}
	GMEDEBUG(GMEDATA_DRV,
		"bdata(HEX)= %02x %02x %02x %02x %02x %02x %02x %02x\n",
		data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

	return GME_D_SUCCESS;
}

/*!
 Set calculated data to device driver.
 @param[in] buf The order of input data depends on driver's specification.
 */
void GME_D_SetYPR(const int buf[GME_YPR_DATA_SIZE])
{
	if (s_fdDev < 0) {
		GMEERROR;
		return;
	}
	if (ioctl(s_fdDev, ECS_IOCTL_SET_YPR_60X, buf) < 0) {
		GMEERROR_STR("ioctl");
	}
}

/*!
 */
int16_t GME_D_GetOpenStatus(int* status)
{
	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	ALOGI("ECS_IOCTL_GET_OPEN_STATUS");
	if (ioctl(s_fdDev, ECS_IOCTL_GET_OPEN_STATUS, status) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
	ALOGI("ECS_IOCTL_GET_OPEN_STATUS complete");	
	return GME_D_SUCCESS;
}

/*!
 */
int16_t GME_D_GetCloseStatus(int* status)
{
	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	ALOGI("ECS_IOCTL_GET_CLOSE_STATUS");
	if (ioctl(s_fdDev, ECS_IOCTL_GET_CLOSE_STATUS, status) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
	ALOGI("ECS_IOCTL_GET_CLOSE_STATUS complete");	
	return GME_D_SUCCESS;
}

/*!
 Set GME E-Compass to the specific mode.
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 @param[in] mode This value should be one of the GME_MODE which is defined in
 header file.
 */
int16_t GME_D_SetMode(const BYTE mode)
{
	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	if (ioctl(s_fdDev, ECS_IOCTL_SET_MODE, &mode) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
	return GME_D_SUCCESS;
}

/*!
 Acquire delay
 @return If this function succeeds, the return value is #GME_D_SUCCESS. Otherwise
 the return value is #GME_D_ERROR.
 @param[out] delay A delay in microsecond.
 */
int16_t GME_D_GetDelay(int64_t delay[GME_NUM_SENSORS])
{
	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	if (ioctl(s_fdDev, ECS_IOCTL_GET_DELAY_60X, delay) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
//	ALOGI("%s:%d:%d delay=%lld,%lld,%lld\n",
//		__func__,__LINE__,__TIME__, delay[0], delay[1], delay[2]);

	GMEDEBUG(GMEDATA_DRV, "%s: delay=%lld,%lld,%lld\n",
		__FUNCTION__, delay[0], delay[1], delay[2]);
	return GME_D_SUCCESS;
}

/*!
 Get layout information from device driver, i.e. platform data.
 */
int16_t GME_D_GetLayout(int16_t* layout)
{
	char tmp;

	if (s_fdDev < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	if (ioctl(s_fdDev, ECS_IOCTL_GET_LAYOUT_60X, &tmp) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}

	*layout = tmp;

	GMEDEBUG(GMEDATA_DRV, "%s: layout=%d\n", __FUNCTION__, tmp);
	return GME_D_SUCCESS;
}

/* Get acceleration data. */
int16_t GME_D_GetAccelerationData(int16_t data[3])
{
	char buf[30];
	unsigned int xyz[3];
	memset(buf,0,30);
	if (dev_gsensor < 0) {
		GMEERROR;
		return GME_D_ERROR;
	}
	
	if (ioctl(dev_gsensor, GSENSOR_IOCTL_READ_SENSORDATA, buf) < 0) {
		GMEERROR_STR("ioctl");
		return GME_D_ERROR;
	}
	//ALOGI("%s:%d GME_D_GetAccelerationData %s\n",__func__,__LINE__,buf);

	sscanf(buf, "%x %x %x",&xyz[0],&xyz[1],&xyz[2]);
	data[0]=(int)xyz[0];
	data[1]=(int)xyz[1];
	data[2]=(int)xyz[2];
	GMEDEBUG(GMEDATA_DRV, "%s: acc=%d, %d, %d\n",
			__FUNCTION__, data[0], data[1], data[2]);

	return GME_D_SUCCESS;
}
