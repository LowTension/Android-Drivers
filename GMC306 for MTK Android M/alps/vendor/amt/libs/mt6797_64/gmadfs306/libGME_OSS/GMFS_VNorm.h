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
#ifndef GMFS_INC_VNORM_H
#define GMFS_INC_VNORM_H

#include "GMFS_Device.h"

/***** Prototype of function **************************************************/
GMLIB_C_API_START
int16 GMFS_VbNorm(
	const	int16	ndata,
	const	GMFVEC	vdata[],
	const	int16	nbuf,
	const	GMFVEC	*o,
	const	GMFVEC	*s,
	const	GMFLOAT	tgt,
	const	int16	nvec,
			GMFVEC	vvec[]
);

int16 GMFS_VbAve(
	const	int16	nvec,
	const	GMFVEC	vvec[],
	const	int16	nave,
			GMFVEC	*vave
);

GMLIB_C_API_END

#endif

