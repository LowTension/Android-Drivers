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
#ifndef GMFS_INC_COMPASS_H
#define GMFS_INC_COMPASS_H

#include "GMFS_Common.h"
#include "GMFS_CSpec.h"

#ifdef WIN32
#include "GME_LinuxDriver.h"
#else
#include "GMFS_Driver.h"
#endif

/****************************************/
/* Include files for GME OSS library.   */
/****************************************/
#include "./libGME_OSS/GMFS_Configure.h"
#include "./libGME_OSS/GMFS_AOC.h"
#include "./libGME_OSS/GMFS_Decomp.h"
#include "./libGME_OSS/GMFS_Device.h"
#include "./libGME_OSS/GMFS_Direction.h"
#include "./libGME_OSS/GMFS_Math.h"
#include "./libGME_OSS/GMFS_VNorm.h"

/*** Constant definition ******************************************************/
#define GME_MAG_SENSE			(1.0)
#define GME_ACC_SENSE			(720)
#define GME_ACC_TARGET			(9.80665f)

/*** Type declaration *********************************************************/
typedef struct _GMSENSOR_DATA{
	GMFLOAT	x;
	GMFLOAT	y;
	GMFLOAT	z;
    int8	status;
} GMSENSOR_DATA;

/*! A parameter structure. */
/* ix*_ : x-bit integer */
/* f**_ : floating value */
/* p**_ : pointer */
/* e**_ : enum */
/* *s*_ : struct */
/* *v*_ : vector (special type of struct) */
/* **a_ : array */
typedef struct _GMEPRMS{

	/* Variables for Decomp. */
	GMFVEC			fva_hdata[GMFS_HDATA_SIZE];
	uint8vec		i8v_asa;

	/* Variables forAOC. */
	GMFS_AOC_VAR	s_aocv;

	/* Variables for Magnetometer buffer. */
	GMFVEC			fva_hvbuf[GMFS_HDATA_SIZE];
	GMFVEC			fv_ho;
	GMFVEC			fv_hs;
	GMFS_PATNO		e_hpat;

	/* Variables for Accelerometer buffer. */
	GMFVEC			fva_avbuf[GMFS_ADATA_SIZE];
	GMFVEC			fv_ao;
	GMFVEC			fv_as;

	/* Variables for Direction. */
	GMFLOAT			f_azimuth;
	GMFLOAT			f_pitch;
	GMFLOAT			f_roll;

	/* Variables for vector output */
	GMFVEC			fv_hvec;
	GMFVEC			fv_avec;
	int16			i16_hstatus;

} GMEPRMS;

#endif

