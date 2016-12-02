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

/******************************************************************************/
/*! Initialize #GMFVEC array.
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] ndata
  @param[out] vdata
 */
int16 GMFS_InitBuffer(
	const	int16	ndata,		/*!< Size of vector buffer */
			GMFVEC	vdata[]		/*!< Vector buffer */
)
{
	int i;

	/* size check */
	if (ndata <= 0) {
		return GMFS_ERROR;
	}

	for (i = 0; i < ndata; i++) {
		vdata[i].u.x = GMFS_INIT_VALUE_F;
		vdata[i].u.y = GMFS_INIT_VALUE_F;
		vdata[i].u.z = GMFS_INIT_VALUE_F;
	}

	return GMFS_SUCCESS;
}

/******************************************************************************/
/*! Shift #GMFVEC array.
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] len
  @param[in] shift
  @param[in/out] v
 */
int16 GMFS_BufShift(
	const	int16	len,	/*!< size of buffer */
	const	int16	shift,	/*!< shift size */
			GMFVEC	v[] /*!< buffer */
)
{
	int16 i;

	if ((shift < 1) || (len < shift)) {
		return GMFS_ERROR;
	}
	for (i = len-1; i >= shift; i--) {
		v[i] = v[i-shift];
	}
	return GMFS_SUCCESS;
}

/******************************************************************************/
/*! Rotate vector according to the specified layout pattern number.
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] pat
  @param[in/out] vec
 */
int16 GMFS_Rotate(
	const	GMFS_PATNO	pat,
			GMFVEC		*vec
)
{
	GMFLOAT tmp;
	switch (pat) {
	/* Obverse */
	case PAT1:								//1 Xg= Xb,Yg= Yb 
		/* This is Android default */
		break;
	case PAT2:								//2 Xg= Yb,Yg=-Xb 
		tmp = vec->u.x;
		vec->u.x = vec->u.y;
		vec->u.y = -tmp;
		break;
	case PAT3:								//3 Xg=-Xb,Yg=-Yb 
		vec->u.x = -(vec->u.x);
		vec->u.y = -(vec->u.y);
		break;
	case PAT4:								//4 Xg=-Yb,Yg= Xb 
		tmp = vec->u.x;
		vec->u.x = -(vec->u.y);
		vec->u.y = tmp;
		break;
	/* Reverse */
	case PAT5:								//-1 Xg=-Xb,Yg= Yb , Zg=-Zb 
		vec->u.x = -(vec->u.x);
		vec->u.z = -(vec->u.z);
		break;
	case PAT8:								//-4 Xg= Yb,Yg= Xb , Zg=-Zb
		tmp = vec->u.x;
		vec->u.x = vec->u.y;
		vec->u.y = tmp;
		vec->u.z = -(vec->u.z);
		break;
	case PAT7:								//-3 Xg= Xb,Yg=-Yb , Zg=-Zb
		vec->u.y = -(vec->u.y);
		vec->u.z = -(vec->u.z);
		break;
	case PAT6:								//-2 Xg=-Yb,Yg=-Xb , Zg=-Zb
		tmp = vec->u.x;
		vec->u.x = -(vec->u.y);
		vec->u.y = -tmp;
		vec->u.z = -(vec->u.z);
		break;
	default:
		return GMFS_ERROR;
	}

	return GMFS_SUCCESS;
}

/******************************************************************************/
/*! Rotate vector according to the specified layout matrix.
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] pat
  @param[in/out] vec
 */
int16 GMFS_RotateMat(
	const   int16		layout[3][3],
			GMFVEC		*vec
)
{
	GMFVEC tmp;

	tmp.u.x = layout[0][0]*vec->u.x + layout[0][1]*vec->u.y + layout[0][2]*vec->u.z;
	tmp.u.y = layout[1][0]*vec->u.x + layout[1][1]*vec->u.y + layout[1][2]*vec->u.z;
	tmp.u.z = layout[2][0]*vec->u.x + layout[2][1]*vec->u.y + layout[2][2]*vec->u.z;

	*vec = tmp;

	return GMFS_SUCCESS;
}
