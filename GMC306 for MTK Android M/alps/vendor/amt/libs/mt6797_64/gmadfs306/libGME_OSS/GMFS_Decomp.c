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
#include "GMFS_Decomp.h"
#include "GMFS_Log.h"
//#include "GMFS_Device.h"

/******************************************************************************/
/*! Convert from sensor local data unit to micro tesla, then buffer the data.
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] mag
  @param[in] status
  @param[in] asa
  @param[in] nhdata
  @param[out] hdata
 */


int16 GMFS_Decomp(
	const	int16		mag[3],
	const	int16		status,
	const	uint8vec	*asa,
	const	int16		nhdata,
			GMFVEC		hdata[]
)
{
	/* put st1 and st2 value */
	if (GME_ST_ERROR(status)) {
		return GMFS_ERROR;
	}

	/* magnetic */
	GMFS_BufShift(nhdata, 1, hdata);

	hdata[0].u.x = GME_HDATA_CONVERTER(mag[0], asa->u.x) * GME_SENSITIVITY;
	hdata[0].u.y = GME_HDATA_CONVERTER(mag[1], asa->u.y) * GME_SENSITIVITY;
	hdata[0].u.z = GME_HDATA_CONVERTER(mag[2], asa->u.z) * GME_SENSITIVITY;
	ALOGI("SOFT,sv_mag=(%8d,%8d,%8.3f,%8.3f)\n",mag[0], asa->u.x,hdata[0].u.x ,GME_SENSITIVITY);

	return GMFS_SUCCESS;
}

