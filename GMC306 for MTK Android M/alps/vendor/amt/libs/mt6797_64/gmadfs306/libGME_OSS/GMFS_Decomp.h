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
#ifndef GMFS_INC_DECOMP_H
#define GMFS_INC_DECOMP_H

#include "GMFS_Device.h"

/***** Constant definition ****************************************************/
#if defined(GME_DEVICE_GME605)

#define GME_SENSITIVITY			0.6f
#define GME_ST_ERROR(st)		(((st)&0x09) != 0x01)
#define GME_HDATA_CONVERTER(data, asa)			\
		(GMFLOAT)(((data)*(((asa)/128.0f) + 1.0f)))
		
#elif defined(GME_DEVICE_GMC303)

#define GME_SENSITIVITY			0.6f
#define GME_ST_ERROR(st)		(((st)&0x09) != 0x01)
#define GME_HDATA_CONVERTER(data, asa)			\
	(GMFLOAT)(((data)*(((asa)/128.0f) + 1.0f)))
	
#elif defined(GME_DEVICE_GMC306)

#define GME_SENSITIVITY			0.15f
#define GME_ST_ERROR(st)		(((st)&0x09) != 0x01)
#define GME_HDATA_CONVERTER(data, asa)			\
		(GMFLOAT)(((data)*(((asa)/128.0f) + 1.0f)))
#endif


/***** Type declaration *******************************************************/

/***** Prototype of function **************************************************/
GMLIB_C_API_START
int16 GMFS_Decomp(
	const	int16		mag[3],
	const	int16		status,
	const	uint8vec	*asa,
	const	int16		nhdata,
			GMFVEC		hdata[]
);
GMLIB_C_API_END

#endif
