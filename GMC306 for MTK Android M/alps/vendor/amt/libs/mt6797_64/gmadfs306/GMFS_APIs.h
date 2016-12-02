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
#ifndef GMFS_INC_APIS_H
#define GMFS_INC_APIS_H

/* Include files for GM8975 library. */
#include "GMFS_Compass.h"

/*** Constant definition ******************************************************/
#define GMFS_GEOMAG_MAX 70

/*** Type declaration *********************************************************/

/*** Global variables *********************************************************/

/*** Prototype of function ****************************************************/
int16 GMFS_Init(
			void		*mem,
	const	GMFS_PATNO	hpat,
	const	uint8		regs[]
);

void GMFS_Release(void *mem);

int16 GMFS_Start(void *mem, const char *path);

int16 GMFS_Stop(void *mem, const char *path);

int16 GMFS_Get_MAGNETIC_FIELD(
			void		*mem,
	const   int16		mag[3],
	const	int16		status,
			GMFLOAT		*hx,
			GMFLOAT		*hy,
			GMFLOAT		*hz,
			int16		*accuracy,
	const	int16		mfreq
);

int16 GMFS_Get_ACCELEROMETER(
			void		*mem,
	const   int16		acc[3],
	const	int16		status,
			GMFLOAT		*ax,
			GMFLOAT		*ay,
			GMFLOAT		*az,
			int16		*accuracy
);

int16 GMFS_Get_ORIENTATION(
			void		*mem,
			GMFLOAT		*azimuth,
			GMFLOAT		*pitch,
			GMFLOAT		*roll,
			int16		*accuracy
);

#endif

