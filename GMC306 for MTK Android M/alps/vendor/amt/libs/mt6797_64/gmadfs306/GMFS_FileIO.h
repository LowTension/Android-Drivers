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
#ifndef GMFS_INC_FILEIO_H
#define GMFS_INC_FILEIO_H

/* Common include files. */
#include "GMFS_Common.h"

/* Include file for GME OSS library. */
#include "GMFS_Compass.h"

/*** Constant definition ******************************************************/

/*** Type declaration *********************************************************/

/*** Global variables *********************************************************/

/*** Prototype of function ****************************************************/
int16 GMFS_LoadParameters(GMEPRMS *prms, const char* path);

int16 GMFS_SaveParameters(GMEPRMS* prms, const char* path);

#endif

