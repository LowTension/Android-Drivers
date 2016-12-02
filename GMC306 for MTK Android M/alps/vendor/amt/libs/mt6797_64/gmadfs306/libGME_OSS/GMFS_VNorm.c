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
#include "GMFS_VNorm.h"

/******************************************************************************/
/*! Normalize vector.
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] ndata Size of raw vector buffer
  @param[in] vdata Raw vector buffer
  @param[in] nbuf Size of data to be buffered
  @param[in] o Offset
  @param[in] s Sensitivity
  @param[in] tgt Target sensitivity
  @param[in] nvec Size of normalized vector buffer
  @param[out] vvec Normalized vector buffer
 */
int16 GMFS_VbNorm(
	const	int16	ndata,
	const	GMFVEC	vdata[],
	const	int16	nbuf,
	const	GMFVEC	*o,
	const	GMFVEC	*s,
	const	GMFLOAT	tgt,
	const	int16	nvec,
			GMFVEC	vvec[]
)
{
	int i;

	/* size check */
	if ((ndata <= 0) || (nvec <= 0) || (nbuf <= 0)) {
		return GMFS_ERROR;
	}
	/* dependency check */
	if ((nbuf < 1) || (ndata < nbuf) || (nvec < nbuf)) {
		return GMFS_ERROR;
	}
	/* sensitivity check */
	if ((s->u.x <= GMFS_EPSILON) ||
		(s->u.y <= GMFS_EPSILON) ||
		(s->u.z <= GMFS_EPSILON) ||
		(tgt <= 0)) {
		return GMFS_ERROR;
	}

	/* calculate and store data to buffer */
	if (GMFS_BufShift(nvec, nbuf, vvec) != GMFS_SUCCESS) {
		return GMFS_ERROR;
	}
	for (i = 0; i < nbuf; i++) {
		vvec[i].u.x = ((vdata[i].u.x - o->u.x) / (s->u.x) * (GMFLOAT)tgt);
		vvec[i].u.y = ((vdata[i].u.y - o->u.y) / (s->u.y) * (GMFLOAT)tgt);
		vvec[i].u.z = ((vdata[i].u.z - o->u.z) / (s->u.z) * (GMFLOAT)tgt);
	}

	return GMFS_SUCCESS;
}

/******************************************************************************/
/*! Calculate an averaged vector form a given buffer.
  @return #GMFS_SUCCESS on success. Otherwise the return value is #GMFS_ERROR.
  @param[in] nvec Size of normalized vector buffer
  @param[in] vvec Normalized vector buffer
  @param[in] nave Number of average
  @param[out] vave Averaged vector
 */
int16 GMFS_VbAve(
	const	int16	nvec,
	const	GMFVEC	vvec[],
	const	int16	nave,
			GMFVEC	*vave
)
{
	int i;

	/* arguments check */
	if ((nave <= 0) || (nvec <= 0) || (nvec < nave)) {
		return GMFS_ERROR;
	}

	/* calculate average */
	vave->u.x = 0;
	vave->u.y = 0;
	vave->u.z = 0;
	for (i = 0; i < nave; i++) {
		if ((vvec[i].u.x == GMFS_INIT_VALUE_F) ||
			(vvec[i].u.y == GMFS_INIT_VALUE_F) ||
			(vvec[i].u.z == GMFS_INIT_VALUE_F)) {
				break;
		}
		vave->u.x += vvec[i].u.x;
		vave->u.y += vvec[i].u.y;
		vave->u.z += vvec[i].u.z;
	}
	if (i == 0) {
		vave->u.x = 0;
		vave->u.y = 0;
		vave->u.z = 0;
	} else {
		vave->u.x /= i;
		vave->u.y /= i;
		vave->u.z /= i;
	}
	return GMFS_SUCCESS;
}


