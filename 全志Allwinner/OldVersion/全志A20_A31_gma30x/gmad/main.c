/* 
 * Copyright (c) 2014 Globalmems, Inc.  All rights reserved.
 *
 * This source is subject to the Globalmems Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of Globalmems Inc.
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * Program:
 *       Globalmems Execute file.(daemon)
 * Usage: 
 *		/system/bin/gmad -c asix
 * 		/system/bin/gmad -r
 * 		/system/bin/gmad -g
 * 		/system/bin/gmad -s offsetX offsetY offsetZ
 * 		/system/bin/gmad -x 
 * Function:
 * 		--calibration (-c)	: calibration & save file into /data/misc/sensor/offset.txt(Driver level)
 * Argument:
 * 		asix: 1		GRAVITY_ON_Z_NEGATIVE
 *			  2		GRAVITY_ON_Z_POSITIVE
 *			  3 	GRAVITY_ON_Y_NEGATIVE
 *			  4 	GRAVITY_ON_Y_POSITIVE
 *			  5 	GRAVITY_ON_X_NEGATIVE
 *			  6 	GRAVITY_ON_X_POSITIVE
 *			  7 	GRAVITY_ON_X_AUTO
 *			  8 	GRAVITY_ON_Y_AUTO
 *			  9 	GRAVITY_ON_Z_AUTO 
 * Function:
 * 		--reset		(-r)	: sensor reset
 * Function:
 * 		--getoffset (-g)	: get offset
 * Function:
 * 		--setoffset (-s)	: set X/Y/Z offset
 * Function:
 * 		--rawdata	(-x)	: read X/Y/Z from chip(rawdata) 1g=64 ( or 128) ref datasheet
 * Function:
 * 		--data		(-d)	: read X/Y/Z from Driver (rawdata - offset) 1g=9807
 * Example Usage:
 * 		/system/bin/gmad -c 9
 * 		/system/bin/gmad --calibration 9 
 * 		/system/bin/gmad -r
 * 		/system/bin/gmad --reset
 * 		/system/bin/gmad -g
 * 		/system/bin/gmad --getoffset
 * 		/system/bin/gmad -s 10 -10 253
 * 		/system/bin/gmad --setoffset 10 -10 253
 * 		/system/bin/gmad -x
 * 		/system/bin/gmad --rawdata
 * 		/system/bin/gmad -d
 * 		/system/bin/gmad --data
 * History:
 * 2014/05/13	Bruce	First release
 * 2014/06/06	Bruce	V0.2
 * 2014/07/24	Bruce	V0.3	Calibration: support RK Driver (IOCTL 'a' & 0xA1)
 * 2014/10/21	Bruce	V0.4	Calibration: add support /dev/gma303
 */
#include "GMACommon.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <linux/input.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <poll.h>
#include <time.h>
#include <cutils/log.h>
#define _GNU_SOURCE
#include <getopt.h>
//public static String TAG = "Sensors";
//#define LOG_TAG "gmad"
//#define GSE_TAG                  "[GMA30x]"

char GMA_OffsetTxt[] = "/data/misc/sensor/offset.txt";	/* FILE offset.txt */
char ATTR_Calibration[] = "/sys/class/input/input0/calibration";	/* calibration input path (check inputX)*/
char ATTR_OFFSET[] = "/sys/class/input/input0/offset";	/* offset input path (check inputX)*/
int open_init(unsigned int *cmd)
{
	int fd = 0;
	if((fd= open(ACC_SENSOR_GMA302,O_RDONLY)) > 0) //gma302
 	{
		*cmd = GlobalMems;
		return fd;
	}
	if((fd= open(ACC_SENSOR_GMA303,O_RDONLY)) > 0) //gma303
 	{
		*cmd = GlobalMems;
		return fd;
	}
	if((fd= open(DEVICE_SENSOR_RK,O_RDONLY)) > 0) //RK platform
 	{
		*cmd = RockChip;
		return fd;
	}
	if((fd= open(DEVICE_SENSOR_MTK,O_RDONLY)) > 0) //MTK platform
 	{
		*cmd = MTK;
		return fd;
	}
	if((fd= open(ACC_SENSOR_GMA301,O_RDONLY)) > 0) //gma301
 	{
		*cmd = GlobalMems;
		return fd;
	}
	return fd;
}

int save_offset(int buffer[SENSOR_DATA_SIZE])
{
	FILE *fp=fopen(GMA_OffsetTxt,"wt"); // open offset
	//fwrite(offset,sizeof(int),SENSOR_DATA_SIZE,fp); //二進制存檔
	fprintf(fp,"%d %d %d", buffer[0], buffer[1], buffer[2]); //格式化資料存檔
	fclose(fp);
	
 	return 0;
}
/*
int calibration_input(int asix)
{
	FILE *fp;
	fp=fopen(ATTR_Calibration,"wt"); // open offset
	if (fp == NULL) 
	{
		perror(ATTR_Calibration);
		exit(EXIT_FAILURE);
	}

	//fwrite(offset,sizeof(int),SENSOR_DATA_SIZE,fp); //二進制存檔
	fprintf(fp,"%d", asix); //格式化資料存檔
	printf("%d : %d", asix, __LINE__);
	fclose(fp);
	
 	return 0;
}
*/
int read_offset_input(int buffer[SENSOR_DATA_SIZE])
{
	int ux, uy, uz;
	FILE *fp=fopen(ATTR_OFFSET,"r"); // open offset
	//fread(offset, sizeof(int),SENSOR_DATA_SIZE,fp);//二進制讀檔
	while (EOF != fscanf(fp,"%d %d %d", &ux,&uy,&uz))//格式化資料讀檔
		printf("%d %d %d", ux, uy, uz);
	
	// fread可以一次把數個bytes的資料讀入記憶體中
	buffer[0]=ux;
	buffer[1]=uy;
	buffer[2]=uz;
	fclose(fp);
 	return 0;
}
int read_offset(int buffer[SENSOR_DATA_SIZE])
{
	int ux, uy, uz;
	FILE *fp=fopen(GMA_OffsetTxt,"r"); // open offset
	//fread(offset, sizeof(int),SENSOR_DATA_SIZE,fp);//二進制讀檔
	while (EOF != fscanf(fp,"%d %d %d", &ux,&uy,&uz))//格式化資料讀檔
		printf("%d %d %d", ux, uy, uz);
  
	// fread可以一次把數個bytes的資料讀入記憶體中
	buffer[0]=ux;
	buffer[1]=uy;
	buffer[2]=uz;
	//for ( i=0; i < SENSOR_DATA_SIZE; i++ )
		//printf("buffer[%d] = %d\n", i, buffer[i]);
	fclose(fp);
 	return 0;
}

int cab(int asix)
{
	int fd, ret, buffer[SENSOR_DATA_SIZE];
	//int get[SENSOR_DATA_SIZE];
	unsigned int magic_cmd = 0;
	//=======for MTK=================尚需平台配合驅動程式來驗證
	int i, j , tmp[3], x, y, z;; 
	static char buf[128]; 
	long xyz_acc[3];
	typedef struct{
	int x;
	int y;
	int z;
	}SENSOR_DATA;
	SENSOR_DATA sensor_data;
	raw_data 	last;			/* RawData */
	raw_data 	offset;			/* Offset */
	/* initialize the accumulation buffer */
	for(i = 0; i < 3; ++i){
		xyz_acc[i] = 0;
		tmp[i] = 0;
	}
	tmp[0] = asix;
	//================================================
	/* 1. check char device */
	fd= open_init(&magic_cmd);
	//printf("magic_cmd = %d\n",magic_cmd);
	if (fd < 0) {
		printf("dev_open failed\n");
		exit(1);
		//goto dev_err;
	}
	/* 2. Calibration IOCTL Path */
	switch(magic_cmd) 
	{
		case AllWinner:
			//ioctl(fd,SENSOR_CALIBRATION, &buffer);
			//ALOD("AllWinner : %d %d %d :line %d", buffer[0], buffer[1], buffer[2], __LINE__);
			break;
		case RockChip: // /dev/mma8452_daemon
#define GSENSOR_IOCTL_MAGIC	'a'	/* support RK driver GSENSOR_IOCTL_MAGIC	'a' */
#define GSENSOR_IO			0xA1/* support RK driver GSENSOR_IO	0xA1	*/
#define GBUFF_SIZE			12	/* Rx buffer size */
/* IOCTLs for MMA8452 library */
#define GSENSOR_IOCTL_INIT			_IO(GSENSOR_IOCTL_MAGIC, 0x01)
#define GSENSOR_IOCTL_RESET			_IO(GSENSOR_IOCTL_MAGIC, 0x04)
#define GSENSOR_IOCTL_CLOSE			_IO(GSENSOR_IOCTL_MAGIC, 0x02)
#define GSENSOR_IOCTL_START			_IO(GSENSOR_IOCTL_MAGIC, 0x03)
#define GSENSOR_IOCTL_CALIBRATION	_IOWR(GSENSOR_IOCTL_MAGIC, 0x05, int[SENSOR_DATA_SIZE])
#define GSENSOR_IOCTL_GETDATA		_IOR(GSENSOR_IOCTL_MAGIC, 0x08, char[GBUFF_SIZE+1])
/* IOCTLs for APPs */
#define GSENSOR_IOCTL_APP_SET_RATE		_IOW(GSENSOR_IOCTL_MAGIC, 0x10, char)
			// for rk 0xA1 test
			buffer[0] = asix;
			ret = ioctl(fd, GSENSOR_IOCTL_CALIBRATION, &buffer); 
			if(ret < 0){
				printf("RockChip ret=%d : %d %d %d :line %d\n", ret, buffer[0], buffer[1], buffer[2], __LINE__);
				ret = ioctl(fd, _IOWR(GSENSOR_IO, 0x05 , int[SENSOR_DATA_SIZE]), &buffer); 
				if(ret < 0)
					printf("RockChip ret=%d : %d %d %d :line %d\n", ret, buffer[0], buffer[1], buffer[2], __LINE__);
			}
			printf("%d %d %d\n", buffer[0], buffer[1], buffer[2]);
			break;
		case GlobalMems: // /dev/gma301 || /dev/gma303
#define GMA_IOCTL  0x99
#define GMA_IOCTL_READ              	_IOWR(GMA_IOCTL, 0x01, char*)
#define GMA_IOCTL_WRITE					_IOW(GMA_IOCTL, 0x02, char*)
#define GMA_IOCTL_SET_MODE				_IOW(GMA_IOCTL, 0x03, short)
#define GMA_IOCTL_RESET					_IO(GMA_IOCTL, 0x04)	//ACC sensor reset
#define GMA_IOCTL_CALIBRATION			_IOWR(GMA_IOCTL, 0x05, int[SENSOR_DATA_SIZE])	//ACC sensor Calibration
#define GMA_IOCTL_GET_OFFSET			_IOR(GMA_IOCTL, 0x06, int[SENSOR_DATA_SIZE])	//GET ACC sensor offset
#define GMA_IOCTL_SET_OFFSET			_IOWR(GMA_IOCTL, 0x07, int[SENSOR_DATA_SIZE])	//SET ACC sensor offset
#define GMA_IOCTL_READ_ACCEL_RAW_XYZ	_IOR(GMA_IOCTL, 0x08, int[SENSOR_DATA_SIZE])	//read sensor rawdata
#define GMA_IOCTL_READ_ACCEL_XYZ		_IOR(GMA_IOCTL, 0x09, int[SENSOR_DATA_SIZE])	//read input report data
#define GMA_IOCTL_SETYPR				_IOW(GMA_IOCTL, 0x0A, int[SENSOR_DATA_SIZE])	//set Yaw-Pitch-Roll
#define GMA_IOCTL_GET_OPEN_STATUS		_IO(GMA_IOCTL, 0x0B)	//get sensor open status
#define GMA_IOCTL_GET_CLOSE_STATUS		_IO(GMA_IOCTL, 0x0C)	//get sensor close status 
#define GMA_IOCTL_GET_DELAY				_IOR(GMA_IOCTL, 0x0D, unsigned int*)
#define GMA_IOCTL_GET_LAYOUT			_IOR(GMA_IOCTL, 0x0E, char)
#define GMA_IOCTL_GET_TEMPERATURE		_IOR(GMA_IOCTL, 0x0F, char)
#define SENSOR_MAXNR 16
			buffer[0] = asix;
			ret = ioctl(fd, GMA_IOCTL_CALIBRATION, &buffer); 
			if(ret < 0){
				printf("GlobalMems ret=%d : %d %d %d :line %d\n", ret, buffer[0], buffer[1], buffer[2], __LINE__);
				//ret = ioctl(fd, _IOWR(GSENSOR_IO, 0x05 , int[SENSOR_DATA_SIZE]), &buffer); 
				//if(ret < 0){
					//printf("RockChip ret=%d : %d %d %d :line %d\n", ret, buffer[0], buffer[1], buffer[2], __LINE__);
				//}
			}
			printf("%d %d %d\n", buffer[0], buffer[1], buffer[2]);

			break;
        case MTK:
/* 		#define GSENSOR						   	0x85
 		#define GSENSOR_IOCTL_INIT                  _IO(GSENSOR,  0x01)
 		#define GSENSOR_IOCTL_READ_CHIPINFO         _IOR(GSENSOR, 0x02, int)
 		#define GSENSOR_IOCTL_READ_SENSORDATA       _IOR(GSENSOR, 0x03, int)
 		#define GSENSOR_IOCTL_READ_OFFSET			_IOR(GSENSOR, 0x04, GSENSOR_VECTOR3D)
 		#define GSENSOR_IOCTL_READ_GAIN				_IOR(GSENSOR, 0x05, GSENSOR_VECTOR3D)
 		#define GSENSOR_IOCTL_READ_RAW_DATA			_IOR(GSENSOR, 0x06, int)
 		#define GSENSOR_IOCTL_SET_CALI				_IOW(GSENSOR, 0x06, SENSOR_DATA)
 		#define GSENSOR_IOCTL_GET_CALI				_IOW(GSENSOR, 0x07, SENSOR_DATA)
 		#define GSENSOR_IOCTL_CLR_CALI				_IO(GSENSOR, 0x08)
 			for(i = 0; i < AVG_NUM; i++)
 			{
 				ret = ioctl(fd, GSENSOR_IOCTL_READ_RAW_DATA, buf);
 				if(ret)
 					printf("read data fail: %s(%d)\n", strerror(errno), errno);
 				else if(3 != sscanf(buf, "DMARD08_ReadRawData %x %x %x", &x, &y, &z))
 					printf("read format fail: %s(%d)\n", strerror(errno), errno);
 				else
 				{
 					tmp[0] = (int)(x);
 					tmp[1] = (int)(y);
 					tmp[2] = (int)(z);
 				}
 				for(j = 0; j < SENSOR_DATA_SIZE; ++j)
 				{
 					xyz_acc[j] += tmp[j];
 					printf("xyz_acc[%d]= %ld", j, xyz_acc[j]);
 				}
 				usleep(17);
 			}

 			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
 				last.v[i] = xyz_acc[i] / AVG_NUM;

 			offset.u.x =  0 - last.u.y ;
 			offset.u.y =  last.u.x ;
 			offset.u.z =  DEFAULT_SENSITIVITY - last.u.z;

 			printf("offset.u.x/y/z = %d %d %d", offset.u.x, offset.u.y, offset.u.z);
			// clear offset 
 			ioctl(fd, GSENSOR_IOCTL_CLR_CALI);

 			sensor_data.x = offset.u.x * GRAVITY_EARTH_1000 / 256;
 			sensor_data.y = offset.u.y * GRAVITY_EARTH_1000 / 256;
 			sensor_data.z = offset.u.z * GRAVITY_EARTH_1000 / 256;
 			printf("sensor_data.x/y/z = %d %d %d", sensor_data.x, sensor_data.y, sensor_data.z);
			buffer[0] = sensor_data.x;
 			buffer[1] = sensor_data.y;
 			buffer[2] = sensor_data.z;
 			//gsensor_set_cali(gsc->fd, &cali);
 			ioctl(fd, GSENSOR_IOCTL_SET_CALI, &sensor_data);
 			//ioctl(fd, GSENSOR_IOCTL_GET_CALI, &tmp);
 			//printf("tmp[0]/[1]/[2] = %d %d %d", tmp[0], tmp[1], tmp[2]);
 			//body[0] = tmp[0] *256/9807;
 			//body[1] = tmp[1] *256/9807;
 			//body[2] = tmp[2] *256/9807;
*/
 		 	break;         
		default:
			ALOGE("%s: Invalid argument", __func__);
			break;
	}
	close(fd);
		
	save_offset(buffer);		/* save offset */
	//read_offset(get);		/* get offset */
	
	return 0;
	
//dev_err:
	/* The second path, Need fine tune this function. */
	//calibration_input(9);		/* calibration */
	//read_offset_input(get);	/* get offset */
	//save_offset(get);			/* save offset */
	
	//return 0;
}

int reset(void)
{
	int fd, ret;
	unsigned int magic_cmd = 0;

	/* 1. check char device */
	fd= open_init(&magic_cmd);
	//printf("magic_cmd = %d\n",magic_cmd);
	if (fd < 0) 
	{
		printf("dev_open failed\n");
		exit(1);
		//goto dev_err;
	}
	ret = ioctl(fd, GMA_IOCTL_RESET); 
	if(ret < 0)
	{
		printf("GlobalMems ret=%d :line %d\n", ret, __LINE__);
	}
	close(fd);	
	return 0;
}

int main(int argc, char *argv[])
{
	int fd, opt, ret, tmp[SENSOR_DATA_SIZE];
	unsigned int magic_cmd = 0;
    struct option longopts[] = {
	{"calibration", 0, NULL, 'c'},	// ACC Sensor Calibration
	{"file", 1, NULL, 'f'},			// Reserve
	{"setoffset", 0, NULL, 's'},	// Set Offset
	{"getoffset", 0, NULL, 'g'},	// Get Offset
	{"reset", 0, NULL, 'r'},		// ACC Sensor Reset
	{"rawdata", 0, NULL, 'x'},		// ACC Sensor X/Y/Z Raw Data(from ASIC)
	{"data", 0, NULL, 'd'},			// ACC Sensor X/Y/Z Data
	{"open", 0, NULL, 'o'},			// Get ACC Sensor Open Status
	{"close", 0, NULL, 'e'},		// Get ACC Sensor Close Status
	{"help", 0, NULL, 'h'},			// Example Usage
	{0,0,0,0}};
	
    while((opt = getopt_long(argc, argv, ":cf:sgrxdoeht", longopts, NULL)) != -1) {
        switch(opt) {
        case 'c':
			cab(atoi(argv[2]));	/* calibration */
			break;
		case 's':
			fd= open_init(&magic_cmd);
			if (fd < 0) 
			{
				printf("dev_open failed\n");
				exit(1);
			}
			tmp[0] = atoi(argv[2]);
			tmp[1] = atoi(argv[3]);
			tmp[2] = atoi(argv[4]); 
			ret = ioctl(fd, GMA_IOCTL_SET_OFFSET, &tmp);	/* set offset */
			save_offset(tmp);						/* save offset */
			close(fd);
            printf("%d %d %d \n", tmp[0], tmp[1], tmp[2]);
            break;
        case 'g':
			fd= open_init(&magic_cmd);
			if (fd < 0) 
			{
				printf("dev_open failed\n");
				exit(1);
			}
			ret = ioctl(fd, GMA_IOCTL_GET_OFFSET, &tmp);	/* get offset */
			close(fd);
            printf("%d %d %d \n", tmp[0], tmp[1], tmp[2]);
            break;
        case 'r':
			reset();
            printf("sensor reset\n");
            break;
		case 'x':
			fd= open_init(&magic_cmd);
			if (fd < 0) 
			{
				printf("dev_open failed\n");
				exit(1);
			}
			ret = ioctl(fd, GMA_IOCTL_READ_ACCEL_RAW_XYZ, &tmp); 
			close(fd);
			printf("%d %d %d \n", tmp[0], tmp[1], tmp[2]);
            break;
		case 'd':
			fd= open_init(&magic_cmd);
			if (fd < 0) 
			{
				printf("dev_open failed\n");
				exit(1);
			}
			ret = ioctl(fd, GMA_IOCTL_READ_ACCEL_XYZ, &tmp);
			close(fd);
			printf("%d %d %d \n", tmp[0], tmp[1], tmp[2]);
            break;
		case 'o':
			fd= open_init(&magic_cmd);
			if (fd < 0) 
			{
				printf("dev_open failed\n");
				exit(1);
			}
			ret = ioctl(fd, GMA_IOCTL_GET_OPEN_STATUS); 
			close(fd);
			printf("option: %cd , GMA_GET_OPEN_STATUS=%d\n", opt, ret);
            break;
		case 'e':
			fd= open_init(&magic_cmd);
			if (fd < 0) 
			{
				printf("dev_open failed\n");
				exit(1);
			}
			ret = ioctl(fd, GMA_IOCTL_GET_CLOSE_STATUS); 
			close(fd);
			printf("GMA_GET_CLOSE_STATUS=%d\n", opt);
            break;
		case 't':
			fd= open_init(&magic_cmd);
			if (fd < 0) 
			{
				printf("dev_open failed\n");
				exit(1);
			}
			ret = ioctl(fd, GMA_IOCTL_GET_TEMPERATURE, &tmp); 
			close(fd);
			printf("GMA_GET_CLOSE_STATUS=%d\n", opt);
            break;
        case 'f':
            printf("filename: %s\n", optarg);
            break;
		case 'h':
			printf("Usage: gmad [OPTION]...\n");
            printf("Example Usage: \n");
			printf(" /system/bin/gmad -c 9\n");
			printf(" /system/bin/gmad --calibration 9\n");
			printf(" /system/bin/gmad -r\n");
			printf(" /system/bin/gmad --reset\n");
			printf(" /system/bin/gmad -g\n");
			printf(" /system/bin/gmad --getoffset\n");
			printf(" /system/bin/gmad -s 10 -10 38\n");
			printf(" /system/bin/gmad --setoffset 10 -10 38\n");
			printf(" /system/bin/gmad -x\n");
			printf(" /system/bin/gmad --rawdata\n");
			printf(" /system/bin/gmad -d\n");
			printf(" /system/bin/gmad --data\n");
			printf(" /system/bin/gmad -h\n");
			printf(" /system/bin/gmad --help\n");
            break;
        case ':':
            //printf("option needs a value\n");
            break;
        case '?':
            //ALOGE("%s: Invalid argument. unknown option: %c", __func__, opt);
            break;

        }
    }
/*
    for(; optind < argc; optind++)
	{
        //printf("argument %d: %s\n", optind,argv[optind]);
		printf("Try `gmad --help' for more information.\n");

	}
*/
	return 0;
}
