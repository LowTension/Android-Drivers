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
 */
#ifndef GMFS_INC_MEASURE_H
#define GMFS_INC_MEASURE_H

/* Include files for GM8975 library. */
#include "GMFS_Compass.h"
/*** Constant definition ******************************************************/
#define GMFS_GEOMAG_MAX	70
#define GMFS_GEOMAG_MIN	10

/*** Type declaration *********************************************************/

/*** Global variables *********************************************************/

/*** Prototype of function ****************************************************/


int16 GMFS_Set_MAGNETIC_FIELD(
			GMEPRMS		*prms,
	const	int16		mag[3],
	const	int16		status,
	const 	int16 		mfreq
);

int16 GMFS_Set_ACCELEROMETER(
			GMEPRMS		*prms,
	const	int16		acc[3],
	const	int16		status
);

int16 GMFS_SelfTest(void);
#endif

