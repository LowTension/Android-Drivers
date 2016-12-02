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
#include "GMFS_Device.h"
#include "GMFS_Direction.h"
#include "GMFS_VNorm.h"
#include "GMFS_Math.h"

/******************************************************************************/
/* About definition of coordinate system and calculated value.
  Coordinate system is right-handed.
  X-Axis: from left to right.
  Y-Axis: from bottom to top.
  Z-Axis: from reverse to obverse.

  azimuth: Rotation around Z axis, with positive values
    when y-axis moves toward the x-axis.
  pitch: Rotation around X axis, with positive values
    when z-axis moves toward the y-axis.
  roll: Rotation around Y axis, with positive values
    when x-axis moves toward the z-axis.
*/


/******************************************************************************/
/*! This function is used internally, so output is RADIAN!
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] avec
  @param[out] pitch
  @param[out] roll
 */
static int16 GMFS_Angle(
	const	GMFVEC		*avec,
			GMFLOAT		*pitch,	/* radian */
			GMFLOAT		*roll	/* radian */
)
{
	GMFLOAT	av;	/* Size of vector */

	av = GMFS_SQRT((avec->u.x)*(avec->u.x) + (avec->u.y)*(avec->u.y) + (avec->u.z)*(avec->u.z));

	if (av < GMFS_EPSILON) {
		return GMFS_ERROR;
	}

	*pitch = GMFS_ASIN(-(avec->u.y) / av);
	*roll  = GMFS_ASIN((avec->u.x) / av);

	return GMFS_SUCCESS;
}

/******************************************************************************/
/*! This function is used internally, so output is RADIAN!
  @return None
  @param[in] hvec
  @param[in] pitch
  @param[in] roll
  @param[out] azimuth
 */
static void GMFS_Azimuth(
	const	GMFVEC		*hvec,
	const	GMFLOAT		pitch,	/* radian */
	const	GMFLOAT		roll,	/* radian */
			GMFLOAT		*azimuth	/* radian */
)
{
	GMFLOAT sinP; /* sin value of pitch angle */
	GMFLOAT cosP; /* cos value of pitch angle */
	GMFLOAT sinR; /* sin value of roll angle */
	GMFLOAT cosR; /* cos value of roll angle */
	GMFLOAT Xh;   /* X axis element of vector which is projected to horizontal plane */
	GMFLOAT Yh;   /* Y axis element of vector which is projected to horizontal plane */

	sinP = GMFS_SIN(pitch);
	cosP = GMFS_COS(pitch);
	sinR = GMFS_SIN(roll);
	cosR = GMFS_COS(roll);

	Yh = -(hvec->u.x)*cosR + (hvec->u.z)*sinR;
	Xh = (hvec->u.x)*sinP*sinR + (hvec->u.y)*cosP + (hvec->u.z)*sinP*cosR;

	/* atan2(y, x) -> divisor and dividend is opposite from mathematical equation. */
	*azimuth = GMFS_ATAN2(Yh, Xh);
}

/******************************************************************************/
/*! Output is DEGREE!
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] nhvec
  @param[in] hvec
  @param[in] hnave
  @param[in] navec
  @param[in] avec
  @param[in] anave
  @param[out] azimuth
  @param[out] pitch
  @param[out] roll
 */
int16 GMFS_Direction(
	const	int16		nhvec,
	const	GMFVEC		hvec[],
	const	int16		hnave,
	const	int16		navec,
	const	GMFVEC		avec[],
	const	int16		anave,
			GMFLOAT		*azimuth,
			GMFLOAT		*pitch,
			GMFLOAT		*roll
)
{
	GMFVEC have, aave;
	GMFLOAT azimuthRad;
	GMFLOAT pitchRad;
	GMFLOAT rollRad;

	/* arguments check */
	if ((nhvec <= 0) || (navec <= 0) || (hnave <= 0) || (anave <= 0)) {
		return GMFS_ERROR;
	}
	if ((nhvec < hnave) || (navec < anave)) {
		return GMFS_ERROR;
	}

	/* average */
	if (GMFS_VbAve(nhvec, hvec, hnave, &have) != GMFS_SUCCESS) {
		return GMFS_ERROR;
	}
	if (GMFS_VbAve(navec, avec, anave, &aave) != GMFS_SUCCESS) {
		return GMFS_ERROR;
	}

	/* calculate pitch and roll */
	if (GMFS_Angle(&aave, &pitchRad, &rollRad) != GMFS_SUCCESS) {
		return GMFS_ERROR;
	}

	/* calculate azimuth */
	GMFS_Azimuth(&have, pitchRad, rollRad, &azimuthRad);

	*azimuth = RAD2DEG(azimuthRad);
	*pitch = RAD2DEG(pitchRad);
	*roll = RAD2DEG(rollRad);

	/* Adjust range of azimuth */
	if (*azimuth < 0) {
		*azimuth += 360.0f;
	}

	return GMFS_SUCCESS;
}


