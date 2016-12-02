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
#ifndef GMFS_INC_AOC_H
#define GMFS_INC_AOC_H
#include "GMFS_Device.h"
#include "ossf_magnetic.h"

/***** Constant definition ****************************************************/
#define GMFS_HBUF_SIZE	20
#define GMFS_HOBUF_SIZE	4
#define GMFS_HR_TH		10
#define GMFS_HO_TH		0.15

/***** Macro definition *******************************************************/

/***** Type declaration *******************************************************/
typedef struct _GMFS_AOC_VAR{
#ifdef OSSF_CALIBRATION
	GMFVEC		hbuf[OSSF_HBUF_SIZE];
#else
	GMFVEC		hbuf[GMFS_HBUF_SIZE];
#endif //OSSF_CALIBRATION
	GMFVEC		hobuf[GMFS_HOBUF_SIZE];
	GMFLOAT		hraoc;
} GMFS_AOC_VAR;

/***** Prototype of function **************************************************/
GMLIB_C_API_START
int16 GMFS_AOC(
			GMFS_AOC_VAR	*haocv,
	const	GMFVEC			*hdata,
			GMFVEC			*ho
);

void GMFS_InitAOC(
			GMFS_AOC_VAR	*haocv
);

GMLIB_C_API_END

#endif

