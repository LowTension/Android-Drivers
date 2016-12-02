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
#ifndef GMFS_INC_DEVICE_H
#define GMFS_INC_DEVICE_H

#include <float.h>
#include "GMFS_Configure.h"

/***** Constant definition ****************************************************/
#define GMFS_ERROR			0
#define GMFS_SUCCESS		1

#define GMFS_HDATA_SIZE		32
#define GMFS_ADATA_SIZE		32

/***** Type declaration *******************************************************/
typedef signed char     int8;
typedef signed short    int16;
typedef unsigned char   uint8;
typedef unsigned short  uint16;


#ifdef GMFS_PRECISION_DOUBLE
typedef	double			GMFLOAT;
#define GMFS_EPSILON	DBL_EPSILON
#define GMFS_FMAX		DBL_MAX
#define GMFS_FMIN		DBL_MIN

#else
typedef	float			GMFLOAT;
#define GMFS_EPSILON	FLT_EPSILON
#define GMFS_FMAX		FLT_MAX
#define GMFS_FMIN		FLT_MIN

#endif

/* Treat maximum value as initial value */
#define GMFS_INIT_VALUE_F	GMFS_FMAX

/***** Vector ****************************************************************/
typedef union _uint8vec {
	struct {
		uint8	x;
		uint8	y;
		uint8	z;
	} u;
	uint8	v[3];
} uint8vec;

typedef union _GMFVEC {
	struct {
		GMFLOAT x;
		GMFLOAT y;
		GMFLOAT z;
	} u;
	GMFLOAT	v[3];
} GMFVEC;
/* Transformation matrix for chip mounting position 
first pin top:Positive  bottom:Negative 
*/
/***** Layout pattern ********************************************************/
typedef enum _GMFS_PATNO {
	PAT_INVALID = 0,
	PAT1,	/* obverse: 1st pin is 1: top/upper-left		(  x,  y,  z) */
	PAT2,	/* obverse: 1st pin is 2: top/upper-right		(  y,-x,  z) */
	PAT3,	/* obverse: 1st pin is 3: top/lower-right		(-x,-y,  z) */
	PAT4,	/* obverse: 1st pin is 4: top/lower-left		(-y,  x,  z) */
	PAT5,	/* reverse: 1st pin is 5: bottom/upper-right 	(-x,  y,-z) */
	PAT6,	/* reverse: 1st pin is 6: bottom/upper-left  	(-y,-x,-z) */
	PAT7,	/* reverse: 1st pin is 7: bottom/lower-left		(  x,-y,-z) */
	PAT8	/* reverse: 1st pin is 8: bottom/lower-right 	(  y,  x,-z) */
} GMFS_PATNO;

/***** Prototype of function **************************************************/
GMLIB_C_API_START
int16 GMFS_InitBuffer(
	const	int16	ndata,		/*!< Size of raw vector buffer */
			GMFVEC	vdata[]		/*!< Raw vector buffer */
);

int16 GMFS_BufShift(
	const	int16	len,
	const	int16	shift,
			GMFVEC	v[]
);

int16 GMFS_Rotate(
	const   GMFS_PATNO	pat,
			GMFVEC		*vec
);

int16 GMFS_RotateMat(
	const   int16		layout[3][3],
			GMFVEC		*vec
);
GMLIB_C_API_END

#endif

