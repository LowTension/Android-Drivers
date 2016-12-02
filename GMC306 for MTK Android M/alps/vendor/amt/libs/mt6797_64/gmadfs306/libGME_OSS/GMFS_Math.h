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
#ifndef GMFS_INC_MATH_H
#define GMFS_INC_MATH_H

#include <math.h>
#include "GMFS_Configure.h"

/***** Constant definition ****************************************************/
#define GMFS_PI			3.141592654f
#define RAD2DEG(rad)	((rad)*180.0f/GMFS_PI)

/***** Macro definition *******************************************************/

#ifdef GMFS_PRECISION_DOUBLE
#define GMFS_SIN(x)			sin(x)
#define GMFS_COS(x)			cos(x)
#define GMFS_ASIN(x)		asin(x)
#define GMFS_ACOS(x)		acos(x)
#define GMFS_ATAN2(y, x)	atan2((y), (x))
#define GMFS_SQRT(x)		sqrt(x)
#else
#define GMFS_SIN(x)			sinf(x)
#define GMFS_COS(x)			cosf(x)
#define GMFS_ASIN(x)		asinf(x)
#define GMFS_ACOS(x)		acosf(x)
#define GMFS_ATAN2(y, x)	atan2f((y), (x))
#define GMFS_SQRT(x)		sqrtf(x)
#endif

#endif

