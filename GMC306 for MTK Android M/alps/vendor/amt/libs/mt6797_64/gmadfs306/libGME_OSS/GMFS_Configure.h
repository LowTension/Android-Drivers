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
#ifndef GMFS_INC_CONFIG_H
#define GMFS_INC_CONFIG_H

/***** Language configuration *************************************************/
#if defined(__cplusplus)
#define GMLIB_C_API_START	extern "C" {
#define GMLIB_C_API_END		}
#else
#define GMLIB_C_API_START
#define GMLIB_C_API_END
#endif

/*! If following line is commented in, double type is used for floating point
   calculation */
/*
#define GMFS_PRECISION_DOUBLE
*/

#endif

