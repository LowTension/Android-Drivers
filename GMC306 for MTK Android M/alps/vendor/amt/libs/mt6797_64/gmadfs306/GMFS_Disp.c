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
#include "GMFS_Disp.h"
#include "GMFS_Common.h"

/*!
 Print startup message to Android Log daemon.
 */
void Disp_StartMessage(void)
{
	ALOGI("GMADFS for Open Source v20151225.");
	ALOGI("Debug: %s", ((ENABLE_GMEDEBUG)?("ON"):("OFF")));
}

/*!
 Print ending message to Android Log daemon.
 */
void Disp_EndMessage(int ret)
{
	ALOGI("GMADFS for Android end (%d).", ret);
}

/*!
 Print result
 */
void Disp_Result(int buf[])
{
	GMEDEBUG(GMEDATA_CONSOLE, "Flag=%d\n", buf[0]);
	GMEDEBUG(GMEDATA_CONSOLE, "Acc(%d):%8.2f, %8.2f, %8.2f\n",
		buf[4], REVERT_ACC(buf[1]), REVERT_ACC(buf[2]), REVERT_ACC(buf[3]));
	GMEDEBUG(GMEDATA_CONSOLE, "Mag(%d):%8.2f, %8.2f, %8.2f\n",
		buf[8], REVERT_MAG(buf[5]), REVERT_MAG(buf[6]), REVERT_MAG(buf[7]));
	GMEDEBUG(GMEDATA_CONSOLE, "Ori(%d)=%8.2f, %8.2f, %8.2f\n",
		buf[8], REVERT_ORI(buf[9]), REVERT_ORI(buf[10]), REVERT_ORI(buf[11]));
}

/*!
 Output main menu to stdout and wait for user input from stdin.
 @return Selected mode.
 */
MODE Menu_Main(void)
{
	char msg[20];
	memset(msg, 0, sizeof(msg));
	GMEDEBUG(GMEDATA_CONSOLE,
	" -------------  GlobalMEMS Daemon Application -------------------- \n"
	"   1. Start measurement. \n"
	"   Q. Quit application. \n"
	" ----------------------------------------------------------------- \n"
	" Please select a number.\n"
	"   ---> ");
	fgets(msg, 10, stdin);
	GMEDEBUG(GMEDATA_CONSOLE, "\n");

	/* BUG : If 2-digits number is input, */
	/*    only the first character is compared. */
	if (!strncmp(msg, "1", 1)) {
		return MODE_Measure;
	} else if (strncmp(msg, "Q", 1) == 0 || strncmp(msg, "q", 1) == 0) {
		return MODE_Quit;
	} else {
		return MODE_ERROR;
	}
}

