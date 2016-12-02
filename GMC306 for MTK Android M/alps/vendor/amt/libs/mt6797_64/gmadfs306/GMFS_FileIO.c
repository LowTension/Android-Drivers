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
#include "GMFS_FileIO.h"

/*** Constant definition ******************************************************/
#ifdef GMFS_PRECISION_DOUBLE
#define GMFS_SCANF_FORMAT	"%63s = %lf"
#else
#define GMFS_SCANF_FORMAT	"%63s = %f"
#endif
#define GMFS_PRINTF_FORMAT	"%s = %f\n"
#define LOAD_BUF_SIZE	64

/*!
 Load parameters from file which is specified with #path.  This function reads 
  data from a beginning of the file line by line, and check parameter name 
  sequentially. In other words, this function depends on the order of eache 
  parameter described in the file.
 @return If function fails, the return value is #GME_ERROR. When function fails,
  the output is undefined. Therefore, parameters which are possibly overwritten
  by this function should be initialized again. If function succeeds, the
  return value is #GME_SUCCESS.
 @param[out] prms A pointer to #GMEPRMS structure. Loaded parameter is
  stored to the member of this structure.
 @param[in] path A path to the setting file.
 */
int16 GMFS_LoadParameters(GMEPRMS * prms, const char* path)
{
	int16 ret;
	char buf[LOAD_BUF_SIZE];
	GMFLOAT tmpF;
	FILE *fp = NULL;
	int tmpD;

	/* Open setting file for read. */
	if ((fp = fopen(path, "r")) == NULL) {
		GMEERROR_STR("fopen");
		return GME_ERROR;
	}

	ret = 1;

	/* Load data to HO */
	if (fscanf(fp, GMFS_SCANF_FORMAT, buf, &tmpF) != 2) {
		ret = 0;
	} else {
		if (strncmp(buf, "HO.x", sizeof(buf)) != 0) {
			ret = 0;
		} else {
			prms->fv_ho.u.x = tmpF;
		}
	}
	if (fscanf(fp, GMFS_SCANF_FORMAT, buf, &tmpF) != 2) {
		ret = 0;
	} else {
		if (strncmp(buf, "HO.y", sizeof(buf)) != 0) {
			ret = 0;
		} else {
			prms->fv_ho.u.y = tmpF;
		}
	}
	if (fscanf(fp, GMFS_SCANF_FORMAT, buf, &tmpF) != 2) {
		ret = 0;
	} else {
		if (strncmp(buf, "HO.z", sizeof(buf)) != 0) {
			ret = 0;
		} else {
			prms->fv_ho.u.z = tmpF;
		}
	}
	if (fscanf(fp, "%63s = %d", buf, &tmpD) != 2) {
		ret = 0;
	} else {
		if (strncmp(buf, "HO.c", sizeof(buf)) != 0) {
			ret = 0;
		} else {
			prms->i16_hstatus= tmpD;
		}
	}
	
	if (fclose(fp) != 0) {
		GMEERROR_STR("fclose");
		ret = 0;
	}

	if (ret == 0) {
		GMEERROR;
		return GME_ERROR;
	}

	return GME_SUCCESS;
}

/*!
 Save parameters to file which is specified with #path. This function saves 
  variables when the offsets of magnetic sensor estimated successfully.
 @return If function fails, the return value is #GME_ERROR. When function fails,
  the parameter file may collapsed. Therefore, the parameters file should be
  discarded. If function succeeds, the return value is #GME_SUCCESS.
 @param[out] prms A pointer to #GMEPRMS structure. Member variables are
  saved to the parameter file.
 @param[in] path A path to the setting file.
 */
int16 GMFS_SaveParameters(GMEPRMS *prms, const char* path)
{
	int16 ret = 1;
	FILE *fp;

	/*Open setting file for write. */
	if ((fp = fopen(path, "w")) == NULL) {
		GMEERROR_STR("fopen");
		return GME_ERROR;
	}

	/* Save data to HO */
	if (fprintf(fp, GMFS_PRINTF_FORMAT, "HO.x", prms->fv_ho.u.x) < 0) { ret = 0; }
	if (fprintf(fp, GMFS_PRINTF_FORMAT, "HO.y", prms->fv_ho.u.y) < 0) { ret = 0; }
	if (fprintf(fp, GMFS_PRINTF_FORMAT, "HO.z", prms->fv_ho.u.z) < 0) { ret = 0; }

	if (fprintf(fp, "%s = %d\n", 		"HO.c", prms->i16_hstatus) < 0) { ret = 0; }
	ALOGI("write i16_hstatus ret=%d",ret);
	if (fclose(fp) != 0) {
		GMEERROR_STR("fclose");
		ret = 0;
	}

	if (ret == 0) {
		GMEERROR;
		return GME_ERROR;
	}

	return GME_SUCCESS;
}

