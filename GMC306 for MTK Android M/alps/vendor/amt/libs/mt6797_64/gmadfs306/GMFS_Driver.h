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
#ifndef GMED_INC_GMED_DRIVER_H
#define GMED_INC_GMED_DRIVER_H

#include <stdint.h>			/* int8_t, int16_t etc. */
#if defined(GME_DEVICE_GME605)
#include "gme60x_compass.h"	/* Device driver */
#elif defined(GME_DEVICE_GMC303)
#include "gme60x_compass.h"	/* Device driver */
#elif defined(GME_DEVICE_GMC306)
#include "gmc306.h"	/* Device driver */

#endif

/*** Constant definition ******************************************************/
#define GME_D_TRUE	1		/*!< Represents true */
#define GME_D_FALSE	0		/*!< Represents false */
#define GME_D_SUCCESS	0		/*!< Represents success.*/
#define GME_D_ERROR	-1		/*!< Represents error. */

/*! 0:Don't Output data, 1:Output data */
#define GME_D_DBG_DATA	1
/*! Typical interval in ns */
#define GME_MEASUREMENT_TIME_NS	((GME_MEASURE_TIME_US) * 1000)


/*** Type declaration *********************************************************/
typedef unsigned char BYTE;


/*** Global variables *********************************************************/

/*** Prototype of Function  ***************************************************/

int16_t GME_D_InitDevice(int*,int*);

void GME_D_DeinitDevice(void);

int16_t GME_D_TxData(
		const BYTE address,
		const BYTE* data,
		const uint16_t numberOfBytesToWrite);

int16_t GME_D_RxData(
		const BYTE address,
		BYTE* data,
		const uint16_t numberOfBytesToRead);

int16_t GME_D_Reset(void);

int16_t GME_D_GetSensorInfo(BYTE data[GME_SENSOR_INFO_SIZE]);

int16_t GME_D_GetSensorConf(BYTE data[GME_SENSOR_CONF_SIZE]);

int16_t GME_D_GetMagneticData(BYTE data[GME_SENSOR_DATA_SIZE]);

void GME_D_SetYPR(const int buf[GME_YPR_DATA_SIZE]);

int16_t GME_D_GetOpenStatus(int* status);

int16_t GME_D_GetCloseStatus(int* status);

int16_t GME_D_SetMode(const BYTE mode);

int16_t GME_D_GetDelay(int64_t delay[GME_NUM_SENSORS]);

int16_t GME_D_GetLayout(int16_t* layout);

int16_t GME_D_GetAccelerationData(int16_t data[3]);

#endif /* GMED_INC_GMED_DRIVER_H */
