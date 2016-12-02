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
#include "GMFS_Common.h"
#include "GMFS_Compass.h"
#include "GMFS_FileIO.h"
#include "GMFS_Measure.h"
#include "GMFS_APIs.h"

/******************************************************************************/
/*! Initialize #GMEPRMS structure and make APIs ready to use. This function
  must be called before application uses any APIs in this file.  If #GMEPRMS
  are already initialized, this function discards all existing data. When APIs
  are not used anymore, #GME_Release function must be called at the end.
  When this function succeeds, form factor number is set to 0.
  @return #GME_SUCCESS on success. #GME_ERROR if an error occurred.
  @param[in/out] mem A pointer to a handler.
  @param[in] hpat Specify a layout pattern number.  The number is determined
  according to the mount orientation of the magnetometer.
  @param[in] regs[3] Specify the ASA values which are read out from
  fuse ROM.  regs[0] is ASAX, regs[1] is ASAY, regs[2] is ASAZ.
 */
int16 GMFS_Init(
			void		*mem,
	const	GMFS_PATNO	hpat,
	const	uint8		regs[]
)
{
	GMEPRMS *prms;
#ifdef GME_VALUE_CHECK
	if (mem == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid mem pointer.", __FUNCTION__);
		return GME_ERROR;
	}
#endif
	GMEDEBUG(GMEDATA_DUMP, "%s: hpat=%d, r[0]=0x%02X, r[1]=0x%02X, r[2]=0x%02X\n",
		__FUNCTION__, hpat, regs[0], regs[1], regs[2]);

	/* Copy pointer */
	prms = (GMEPRMS *)mem;

	/* Clear all data. */
	memset(prms, 0, sizeof(GMEPRMS));

	/* Sensitivity */
	prms->fv_hs.u.x = GME_MAG_SENSE;
	prms->fv_hs.u.y = GME_MAG_SENSE;
	prms->fv_hs.u.z = GME_MAG_SENSE;
	prms->fv_as.u.x = GME_ACC_SENSE;
	prms->fv_as.u.y = GME_ACC_SENSE;
	prms->fv_as.u.z = GME_ACC_SENSE;

	/* Copy ASA values */
	prms->i8v_asa.u.x = regs[0];
	prms->i8v_asa.u.y = regs[1];
	prms->i8v_asa.u.z = regs[2];

	/* Copy layout pattern */
	prms->e_hpat = hpat;

	return GME_SUCCESS;
}

/******************************************************************************/
/*! Release allocated memory. This function must be called at the end of using
  APIs. Currently this function is empty.
  @return None
  @param[in/out] mem A pointer to a handler.
 */
void GMFS_Release(void *mem)
{
#ifdef GME_VALUE_CHECK
	if (mem == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid mem pointer.", __FUNCTION__);
		return;
	}
#endif
	/* Do nothing */
}

/******************************************************************************/
/* This function is called just before a measurement sequence starts.
  Load parameters from a file and initialize library. This function must be
  called when a sequential measurement thread boots up.
  @return The return value is #GME_SUCCESS.
  @param[in/out] mem A pointer to a handler.
  @param[in] path The path to a setting file to be read out. The path name
  should be terminated with NULL.
 */
int16 GMFS_Start(void *mem, const char *path)
{
	GMEPRMS *prms;
	ALOGI("%s:%d,GMFS_Start\n",__func__,__LINE__);
#ifdef GME_VALUE_CHECK
	if (mem == NULL || path == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid mem pointer.", __FUNCTION__);
		return GME_ERROR;
	}
#endif
	GMEDEBUG(GMEDATA_DUMP, "%s: path=%s\n", __FUNCTION__, path);

	/* Copy pointer */
	prms = (GMEPRMS *)mem;
	ALOGI("%s:%d,GMFS_LoadParameters\n",__func__,__LINE__);

	/* Read setting files from a file */
	if (GMFS_LoadParameters(prms, path) != GME_SUCCESS) {
		GMEERROR_STR("GMFS_LoadParameters");
	}

	ALOGI("%s:%d,GMFS_InitBuffer\n",__func__,__LINE__);
	/* Initialize buffer */
	GMFS_InitBuffer(GMFS_HDATA_SIZE, prms->fva_hdata);
	GMFS_InitBuffer(GMFS_HDATA_SIZE, prms->fva_hvbuf);
	GMFS_InitBuffer(GMFS_ADATA_SIZE, prms->fva_avbuf);

	/* Initialize for AOC */
	ALOGI("%s:%d,GMFS_InitAOC\n",__func__,__LINE__);
	GMFS_InitAOC(&prms->s_aocv);
	/* Initialize magnetic status */
	//prms->i16_hstatus = 0;

	return GME_SUCCESS;
}

/******************************************************************************/
/*! This function is called when a measurement sequence is done.
  Save parameters to a file. This function must be called when a sequential
  measurement thread ends.
  @return The return value is #GME_SUCCESS.
  @param[in/out] mem A pointer to a handler.
  @param[in] path The path to a setting file to be written. The path name
  should be terminated with NULL.
 */
int16 GMFS_Stop(void *mem, const char *path)
{
	GMEPRMS *prms;
#ifdef GME_VALUE_CHECK
	if (mem == NULL || path == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid mem pointer.", __FUNCTION__);
		return GME_ERROR;
	}
#endif
	GMEDEBUG(GMEDATA_DUMP, "%s: path=%s\n", __FUNCTION__, path);

	/* Copy pointer */
	prms = (GMEPRMS *)mem;

	/* Write setting files to a file */
	if (GMFS_SaveParameters(prms, path) != GME_SUCCESS) {
		GMEERROR_STR("GMFS_SaveParameters");
	}

	return GME_SUCCESS;
}

/*!
  This function is called when new magnetometer data is available.  The output
  vector format and coordination system follow the Android definition.
  @return The return value is #GME_SUCCESS.
   Otherwise the return value is #GME_ERROR.
  @param[in/out] mem A pointer to a handler.
  @param[in] mag A set of measurement data from magnetometer.  X axis value
   should be in mag[0], Y axis value should be in mag[1], Z axis value should be 
   in mag[2].
  @param[in] status A status of magnetometer.  This status indicates the result
   of measurement data, i.e. overflow, success or fail, etc.
 */
int16 GMFS_Get_MAGNETIC_FIELD(
			void		*mem,
	const   int16		mag[3],
	const	int16		status,
			GMFLOAT		*hx,
			GMFLOAT		*hy,
			GMFLOAT		*hz,
			int16		*accuracy,
	const	int16		mfreq
)
{
	GMEPRMS *prms;
#ifdef GME_VALUE_CHECK
	if (mem == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid mem pointer.", __FUNCTION__);
		return GME_ERROR;
	}
	if (hx == NULL || hy == NULL || hz == NULL || accuracy == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid data pointer.", __FUNCTION__);
		return GME_ERROR;
	}
#endif

	/* Copy pointer */
	prms = (GMEPRMS *)mem;

	if (GMFS_Set_MAGNETIC_FIELD(prms, mag, status,mfreq) != GME_SUCCESS) {
		return GME_ERROR;
	}

	/* Success */
	*hx = prms->fv_hvec.u.x;
	*hy = prms->fv_hvec.u.y;
	*hz = prms->fv_hvec.u.z;
	*accuracy = prms->i16_hstatus;

	return GME_SUCCESS;
}

/*!
  This function is called when new accelerometer data is available.  The output
  vector format and coordination system follow the Android definition.
  @return The return value is #GME_SUCCESS when function succeeds. Otherwise
   the return value is #GME_ERROR.
  @param[in] acc A set of measurement data from accelerometer.  X axis value
   should be in acc[0], Y axis value should be in acc[1], Z axis value should be 
   in acc[2].
  @param[in] status A status of accelerometer.  This status indicates the result
   of acceleration data, i.e. overflow, success or fail, etc.
 */
int16 GMFS_Get_ACCELEROMETER(
			void		*mem,
	const   int16		acc[3],
	const	int16		status,
			GMFLOAT		*ax,
			GMFLOAT		*ay,
			GMFLOAT		*az,
			int16		*accuracy
)
{
	GMEPRMS *prms;
#ifdef GME_VALUE_CHECK
	if (mem == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid mem pointer.", __FUNCTION__);
		return GME_ERROR;
	}
	if (ax == NULL || ay == NULL || az == NULL || accuracy == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid data pointer.", __FUNCTION__);
		return GME_ERROR;
	}
#endif

	/* Copy pointer */
	prms = (GMEPRMS *)mem;

	if (GMFS_Set_ACCELEROMETER(prms, acc, status) != GME_SUCCESS) {
		return GME_ERROR;
	}

	/* Success */
	*ax = prms->fv_avec.u.x;
	*ay = prms->fv_avec.u.y;
	*az = prms->fv_avec.u.z;
	*accuracy = 3;

	return GME_SUCCESS;
}

/*!
  Get orientation sensor's elements. The vector format and coordination system
   follow the Android definition.  Before this function is called, magnetic
   field vector and acceleration vector should be stored in the buffer by 
   calling #GMFS_Get_MAGNETIC_FIELD and #GMFS_Get_ACCELEROMETER.
  @return The return value is #GME_SUCCESS when function succeeds. Otherwise
   the return value is #GME_ERROR.
  @param[out] azimuth Azimuthal angle in degree.
  @param[out] pitch Pitch angle in degree.
  @param[out] roll Roll angle in degree.
  @param[out] accuracy Accuracy of orientation sensor.
 */
int16 GMFS_Get_ORIENTATION(
			void		*mem,
			GMFLOAT		*azimuth,
			GMFLOAT		*pitch,
			GMFLOAT		*roll,
			int16		*accuracy
)
{
	int16 akret;
	GMEPRMS *prms;
#ifdef GME_VALUE_CHECK
	if (mem == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid mem pointer.", __FUNCTION__);
		return GME_ERROR;
	}
	if (pitch== NULL || pitch == NULL || roll == NULL || accuracy == NULL) {
		GMEDEBUG(GMEDATA_CHECK, "%s: Invalid data pointer.", __FUNCTION__);
		return GME_ERROR;
	}
#endif

	/* Copy pointer */
	prms = (GMEPRMS *)mem;

	/* Azimuth calculation */
	/* hvbuf[in] : Android coordinate, sensitivity adjusted, */
	/*			   offset subtracted. */
	/* avbuf[in] : Android coordinate, sensitivity adjusted, */
	/*			   offset subtracted. */
	/* azimuth[out]: Android coordinate and unit (degree). */
	/* pitch  [out]: Android coordinate and unit (degree). */
	/* roll   [out]: Android coordinate and unit (degree). */
	akret = GMFS_Direction(
		GMFS_HDATA_SIZE,
		prms->fva_hvbuf,
		CSPEC_HNAVE_D,
		GMFS_ADATA_SIZE,
		prms->fva_avbuf,
		CSPEC_ANAVE_D,
		&prms->f_azimuth,
		&prms->f_pitch,
		&prms->f_roll
	);

	if (akret == GMFS_ERROR) {
		GMEERROR;
		return GME_ERROR;
	}

	/* Success */
	*azimuth = prms->f_azimuth;
	*pitch   = prms->f_pitch;
	*roll    = prms->f_roll;
	*accuracy = prms->i16_hstatus;

	/* Debug output */
	GMEDEBUG(GMEDATA_ORI, "Ori(?):%8.2f, %8.2f, %8.2f\n",
			*azimuth, *pitch, *roll);

	return GME_SUCCESS;
}

