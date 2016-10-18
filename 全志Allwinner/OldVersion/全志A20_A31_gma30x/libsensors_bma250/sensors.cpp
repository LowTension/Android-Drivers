/*
* Copyright (C) Bosch Sensortec GmbH 2011
* Copyright (C) 2008 The Android Open Source Project
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*	   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <linux/input.h>
#include <cutils/atomic.h>
#include <cutils/log.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <cutils/misc.h>

extern "C" int init_module(void *, unsigned long, const char *);
extern "C" int delete_module(const char *, unsigned int);

static int open_sensors(const struct hw_module_t* module, const char* name,struct hw_device_t** device);
static int gsensor_load_driver(struct sensors_poll_context_t *dev);

//#define DEBUG_SENSOR 1

#define NUMOFACCDATA                 8
#define LSG_LIS3DH                  (15.0f)
#define LSG_MMA7660                 (21.0f)
#define LSG_MXC622X                 (64.0f)
#define LSG_BMA250                  (256.0f)
#define LSG_DMARD10                 (256.0f)
#define LSG_DMARD06                 (256.0f)
#define LSG_DMARD09                 (1024.0f)
#define LSG_MMA8452                 (1024.0f)
#define LSG_GMA302                  (1024.0f)
#define LSG_GMA303                  (1024.0f)
#define LSG_KXTIK                   (1024.0f)
#define LSG_AFA750                  (4096.0f)
#define LSG_MMA865X                 (1024.0f)
#define RANGE_A                     (2*GRAVITY_EARTH)
#define RESOLUTION_A                (RANGE_A/(256*NUMOFACCDATA))



#define GSENSOR_NAME "gsensor_name"
#define GSENSOR_DIRECTX "gsensor_direct_x"
#define GSENSOR_DIRECTY "gsensor_direct_y"
#define GSENSOR_DIRECTZ "gsensor_direct_z"
#define GSENSOR_XY "gsensor_xy_revert"

#define TRUE "true"
#define FALSE "false"

#define INPUT_DIR               "/dev/input"
#define GSENSOR_CONFIG_PATH    "/system/usr/gsensor.cfg"
#define LINE_LENGTH  (128)
#define SENSORS_NUMBER    10

static float convert = 0.0;
static float direct_x = 0;
static float direct_y = 0;
static float direct_z = 0;

static int direct_xy = -1;
static int class_value = 0;
static int detect_num = 0;
static int detect_value[5] = {0};



static char class_path[256] = {'\0'};
static char sensor_name[128] ={'\0'};
static char val[LINE_LENGTH] = {'\0'};
   
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))


struct sensors_poll_context_t {
	struct sensors_poll_device_t device; 
	int fd;
};

struct name{
             char name[64];
             float lsg;
};
const struct name sensors[SENSORS_NUMBER] = {
        {"bma250",LSG_BMA250},
        {"mma7660",LSG_MMA7660},
        {"dmard06",LSG_DMARD06},
        {"mma8452",LSG_MMA8452},
        {"kxtik",LSG_KXTIK},
        {"mxc622x",LSG_MXC622X},
        {"afa750",LSG_AFA750},
        {"mma865x",LSG_MMA865X},
        {"lis35de",LSG_LIS3DH},
		{"dmard10",LSG_DMARD10},
		{"dmard09",LSG_DMARD09},
		{"gma302",LSG_GMA302},
		{"gma303",LSG_GMA303}
};

static void get_cfg_value(char *buf)
{
        unsigned int j = 0;
        unsigned int k = 0; 
        memset(&val,0,sizeof(val));
        while(1){
                if(buf[j++] == '='){
                        break;
                }
        }
     
        for( ;j<strlen(buf);j++)
        {
         
                if((buf[j]== ' ')||(buf[j]== '=')||(buf[j]== '\n') || (buf[j]== '\r')||(buf[j]== '\t')) {
         
                }else{
                        val[k++] = buf[j];
                }           
        } 
        val[k] = '\0';
}

static int gsensor_cfg(void)
{
        FILE *fp;
        int name_match = 0;
        char buf[LINE_LENGTH] = {0};
        if((fp = fopen(GSENSOR_CONFIG_PATH,"rb")) == NULL) {
                ALOGD("can't not open file!\n");
                return 0;
        }
   
        while(fgets(buf, LINE_LENGTH , fp)){
                if (!strncmp(buf,GSENSOR_NAME,strlen(GSENSOR_NAME))){
                        
                        get_cfg_value(buf);
                        
                        #ifdef DEBUG_SENSOR
                                ALOGD("val:%s\n",val);
                        #endif
                
                        name_match = (strncmp(val,sensor_name,strlen(sensor_name))) ? 0 : 1;
                        
                        if (name_match)  {
                                 convert = (GRAVITY_EARTH/sensors[class_value].lsg);
                                 #ifdef DEBUG_SENSOR
                                        ALOGD("lsg: %f,convert:%f",sensors[class_value].lsg,convert);
                                 #endif
                                 memset(&buf,0,sizeof(buf));
                                 continue;
                        } 
                }  
                if(name_match ==0){
                        memset(&buf,0,sizeof(buf));
                        continue;
                }else if(name_match < 5){
                        name_match++;
                        get_cfg_value(buf); 
                        
                        #ifdef DEBUG_SENSOR
                                ALOGD("val:%s\n",val);
                        #endif
                       if (!strncmp(buf,GSENSOR_DIRECTX,strlen(GSENSOR_DIRECTX))){                                
                                  direct_x = (strncmp(val,TRUE,strlen(val))) ? convert * (-1) : convert;      
                       }
                       if (!strncmp(buf,GSENSOR_DIRECTY,strlen(GSENSOR_DIRECTY))){                                         
                                  direct_y =(strncmp(val,TRUE,strlen(val))) ? convert * (-1) : convert;      
                       }
                       if (!strncmp(buf,GSENSOR_DIRECTZ,strlen(GSENSOR_DIRECTZ))){
                                 direct_z =(strncmp(val,TRUE,strlen(val))) ? convert * (-1) : convert; 
                       }
                       if (!strncmp(buf,GSENSOR_XY,strlen(GSENSOR_XY))){
                                 direct_xy = (strncmp(val,TRUE,strlen(val))) ? 0 : 1; 
                       }
                               
                        
                }else{
                       name_match = 0;
                       break;
                }
                memset(&buf,0,sizeof(buf));
        }
#ifdef DEBUG_SENSOR
          ALOGD("direct_x: %f,direct_y: %f,direct_z: %f,direct_xy:%d,sensor_name:%s \n",\
          direct_x,direct_y,direct_z,direct_xy,sensor_name);
#endif
        if((direct_x == 0)||(direct_y == 0)||(direct_z == 0)||(direct_xy == (-1))||(sensor_name == NULL)||(convert == 0.0)){
                return 0;
        }
        fclose(fp);
        return 1;
    
}


static int set_sysfs_input_attr(char *class_path,const char *attr, char *value, int len)
{
	char path[256];
	int fd;

	if (class_path == NULL || *class_path == '\0'
	    || attr == NULL || value == NULL || len < 1) {
		return -EINVAL;
	}
	snprintf(path, sizeof(path), "%s/%s", class_path, attr);
	path[sizeof(path) - 1] = '\0';
	fd = open(path, O_RDWR);
	if (fd < 0) {
		return -errno;
	}
	if (write(fd, value, len) < 0) {
		close(fd);
		return -errno;
	}
	close(fd);

	return 0;
}

static int open_input_device(void)
{
	char *filename;
	int fd;
	DIR *dir;
	struct dirent *de;
	char name[80];
	char devname[256];
	dir = opendir(INPUT_DIR);
	if (dir == NULL)
		return -1;

	strcpy(devname, INPUT_DIR);
	filename = devname + strlen(devname);
	*filename++ = '/';

	while ((de = readdir(dir))) {
		if (de->d_name[0] == '.' &&
		    (de->d_name[1] == '\0' ||
		     (de->d_name[1] == '.' && de->d_name[2] == '\0')))
			continue;
		strcpy(filename, de->d_name);
		fd = open(devname, O_RDONLY);
		if (fd < 0) {
			continue;
		}



		if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
			name[0] = '\0';
		}

		if (!strcmp(name, sensor_name)) {
#ifdef DEBUG_SENSOR
		ALOGD("devname is %s \n", devname);
		ALOGD("sensor_name is %s \n", sensor_name);
#endif
		} else {
			close(fd);
			continue;
		}
		closedir(dir);

		return fd;

	}
	closedir(dir);

	return -1;
}

static int sensor_get_class_path(void)
{
	char *dirname =(char *) "/sys/class/input";
	char buf[256];
	int res;
	DIR *dir;
	struct dirent *de;
	int fd = -1;
	int found = 0;
	int ret = 0;
    
        memset(&buf,0,sizeof(buf));
    
	dir = opendir(dirname);
	if (dir == NULL)
		return -1;

	while((de = readdir(dir))) {
    		if (strncmp(de->d_name, "input", strlen("input")) != 0) {
    		        continue;
                }
    
    		sprintf(class_path, "%s/%s", dirname, de->d_name);
    		snprintf(buf, sizeof(buf), "%s/name", class_path);
    
    		fd = open(buf, O_RDONLY);
    		if (fd < 0) {
    		    continue;
    		}
    		if ((res = read(fd, buf, sizeof(buf))) < 0) {
    		    close(fd);
    		    continue;
    		}
    		buf[res - 1] = '\0';    
        	for(class_value = 0; class_value < SENSORS_NUMBER; class_value++ ){
		        if (!strncmp(buf, sensors[class_value].name, strlen(sensors[class_value].name))) {		                            	                             
                                found = 1;
                            	close(fd);
                            	break;
        	        }

    		}
    		if(class_value != SENSORS_NUMBER){
    		           break;
    		}
    		close(fd);
    		fd = -1;
	}
	closedir(dir);

	if (found) { 
	    strcpy(sensor_name,buf);	    
	    ALOGD("class path :%s,sensor_name:%s\n",class_path,sensor_name);
		return 0;
	}else {
	    strcpy(class_path,"\0");
		return -1;
	}


}

static const struct sensor_t sSensorList_kxtik[] = {
        {"Kionix 3-axis Accelerometer",
        "Kionix, Inc.",
        1, 
        0,
        SENSOR_TYPE_ACCELEROMETER,
        RANGE_A,
        RESOLUTION_A,
        0.23f,
        20000,
        { } 
        },
};

static const struct sensor_t sSensorList_mxc622x[] = {
        { 
        "mxc622x 2-axis Accelerometer",
        "MEMSIC",
        1, 
        0,
        SENSOR_TYPE_ACCELEROMETER,
        2.0,
        GRAVITY_EARTH/64.0f,
        0.005,
        0,
        { } 
        },
};
static const struct sensor_t sSensorList_afa750[] = {

        {"AFA750 3-axis Accelerometer",
        "AFA",
        1, 
        0,
        SENSOR_TYPE_ACCELEROMETER,
        4.0f*9.81f,
        GRAVITY_EARTH/4096.0f,
        0.8f,
        0,
        { } 
        },
};





static const struct sensor_t sSensorList_bma250[] = {

        {"Bosch 3-axis Accelerometer",
        "Bosch",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER, 
        4.0f*9.81f, 
        (4.0f*9.81f)/1024.0f, 
        0.2f, 
        0, 
        { } 
        },

};

static const struct sensor_t sSensorList_dmard06[] = {
        { "DMARD06 3-axis Accelerometer",
        "DMT",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER, 
	(GRAVITY_EARTH * 3.0f), 
	GRAVITY_EARTH/256.0f, 
	0.5f, 
	0, 
	{ } 
	},
};
static const struct sensor_t sSensorList_dmard09[] = {
        { "DMARD09 3-axis Accelerometer",
        "DMT",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER, 
	(GRAVITY_EARTH * 3.0f), 
	GRAVITY_EARTH/1024.0f, 
	0.125f, 
	0, 
	{ } 
	},
};
static const struct sensor_t sSensorList_dmard10[] = {
        { "DMARD10 3-axis Accelerometer",
        "DMT",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER, 
	(GRAVITY_EARTH * 2.0f), 
	GRAVITY_EARTH/256.0f,
	0.145f, 

	0, 
	{ } 
	},
};
static const struct sensor_t sSensorList_mma7660[] = {
        { "MMA7660 3-axis Accelerometer",
        "Freescale Semiconductor Inc.",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER,
        RANGE_A, 
        GRAVITY_EARTH/21.0f, 
        0.30f,
        20000,
        { } },
};
static const struct sensor_t sSensorList_mma8452[] = {
        { "MMA8452 3-axis Accelerometer",
        "Freescale Semiconductor Inc.",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER,
        RANGE_A, 
        GRAVITY_EARTH/1024.0f, 
        0.30f,
        20000,
        { } },
};
static const struct sensor_t sSensorList_mma865x[] = {
        { "MMA865x 3-axis Accelerometer",
        "Freescale Semiconductor Inc.",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER,
        RANGE_A, 
        GRAVITY_EARTH/1024.0f, 
        0.30f,
        20000,
        { } },
};

static const struct sensor_t sSensorList_lis3dh[] = {
        
        {"lis3dh detect Accelerometer",
        "ST",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER, 
        4.0f*9.81f, 
        (4.0f*9.81f)/1024.0f, 
        0.2f, 
        0, 
        { } 
        },
};

static const struct sensor_t sSensorList_gsensor[] = {
        
        {"gsensor detect Accelerometer",
        "SW",
        1, 0,
        SENSOR_TYPE_ACCELEROMETER, 
        4.0f*9.81f, 
        (4.0f*9.81f)/1024.0f, 
        0.2f, 
        0, 
        { } 
        },

};







static int sensors__get_sensors_list(struct sensors_module_t* module,struct sensor_t const** list)
{
   int list_value = 0;
   list_value =  class_value;
    switch(list_value)
    {
        case 0:
               *list = sSensorList_bma250;
               return ARRAY_SIZE(sSensorList_bma250);
               break;              
        case 1:
              *list = sSensorList_mma7660;
               return ARRAY_SIZE(sSensorList_mma7660);
               break;
        case 2:
               *list = sSensorList_dmard06;
               return ARRAY_SIZE(sSensorList_dmard06);
               break;
        case 3:
               *list = sSensorList_mma8452;
               return ARRAY_SIZE(sSensorList_mma8452);
               break;
        case 4:
               *list = sSensorList_kxtik;
               return ARRAY_SIZE(sSensorList_kxtik);
               break;
        case 5:
               *list = sSensorList_mxc622x;
               return ARRAY_SIZE(sSensorList_mxc622x);
                break;
        case 6:
               *list = sSensorList_afa750;
               return ARRAY_SIZE(sSensorList_afa750);
                break;
	case 7:
               *list = sSensorList_mma865x;
               return ARRAY_SIZE(sSensorList_mma865x);
                break;
	case 8:
		      *list = sSensorList_lis3dh;
              return ARRAY_SIZE(sSensorList_lis3dh);
			  break;
		case 9:
               *list = sSensorList_dmard10;
               return ARRAY_SIZE(sSensorList_dmard10);
               break;
	case 10:
               *list = sSensorList_dmard09;
               return ARRAY_SIZE(sSensorList_dmard09);
               break;
        default:
                *list = sSensorList_gsensor;
            break;
    }

    return ARRAY_SIZE(sSensorList_gsensor);
    
}

static struct hw_module_methods_t sensors_module_methods = {
	open : open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
	common :{
		tag : HARDWARE_MODULE_TAG,
		version_major : 1,
		version_minor : 0,
		id : SENSORS_HARDWARE_MODULE_ID,
		name : "gsensor module",
		author : "gSensortec",
		methods : &sensors_module_methods,
		dso : NULL,
		reserved : {},
	},

	get_sensors_list : sensors__get_sensors_list
};



static int poll__close(struct hw_device_t *dev)
{
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	if (ctx) {
		delete ctx;
	}

	return 0;
}

static int poll__activate(struct sensors_poll_device_t *device,
        int handle, int enabled) {

	sensors_poll_context_t *dev = (sensors_poll_context_t *)device;
	char buffer[20];

	int bytes = sprintf(buffer, "%d\n", enabled);   
	set_sysfs_input_attr(class_path,"enable",buffer,bytes);

	return 0;
}





static int poll__setDelay(struct sensors_poll_device_t *device,
        int handle, int64_t ns) {

	sensors_poll_context_t *dev = (sensors_poll_context_t *)device;
	char buffer[20];
	int ms=ns/1000000;
	int bytes = sprintf(buffer, "%d\n", ms);

   
	set_sysfs_input_attr(class_path,"delay",buffer,bytes);

	return 0;

}

static int poll__poll(struct sensors_poll_device_t *device,
        sensors_event_t* data, int count) {
	
	struct input_event event;
	int ret;
	sensors_poll_context_t *dev = (sensors_poll_context_t *)device;
  
        if (dev->fd < 0){ 	
                return 0;              
        }

	while (1) {
	
	        ret = read(dev->fd, &event, sizeof(event));
                if(ret < 0){
                        return 0;              
                }
		if (event.type == EV_ABS) {

			switch (event.code) {
			case ABS_X:
			        if(direct_xy) {
			                data->acceleration.y = event.value * direct_y;
			        }else {
				        data->acceleration.x = event.value * direct_x;
			        }
				break;
			case ABS_Y:
			        if(direct_xy) {
			                data->acceleration.x = event.value * direct_x;
			        }else {
			                data->acceleration.y = event.value * direct_y;
				}		
				break;
			
			case ABS_Z:
				data->acceleration.z = event.value * direct_z ;
				break;
			}
		} else if (event.type == EV_SYN) {

			data->timestamp =
			(int64_t)((int64_t)event.time.tv_sec*1000000000
					+ (int64_t)event.time.tv_usec*1000);
			data->sensor = 0;
			data->type = SENSOR_TYPE_ACCELEROMETER;
			data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;			
		
#ifdef DEBUG_SENSOR
			ALOGD("Sensor data: t x,y,z: %f %f, %f, %f\n",
					data->timestamp / 1000000000.0,
							data->acceleration.x,
							data->acceleration.y,
							data->acceleration.z);
#endif
			
		return 1;	


		}
	}
	
	return 0;
}




static int open_sensors(const struct hw_module_t* module, const char* name,
        struct hw_device_t** device)
{ 
	int status = -EINVAL;
    int ret = -1;   

	sensors_poll_context_t *dev = new sensors_poll_context_t();
	memset(&dev->device, 0, sizeof(sensors_poll_device_t));
	
 	if(sensor_get_class_path() < 0) {
		ALOGD("sensor get class path error \n");
		return -1;
	}

        if(!(ret = gsensor_cfg())){
                ALOGD("gsensor config error!\n");
                return -1;
        }
        dev->fd = open_input_device(); 

	
	dev->device.common.tag = HARDWARE_DEVICE_TAG;
	dev->device.common.version  = 0;
	dev->device.common.module   = const_cast<hw_module_t*>(module);
	dev->device.common.close    = poll__close;
	dev->device.activate        = poll__activate;
	dev->device.setDelay        = poll__setDelay;
	dev->device.poll            = poll__poll;

	          
	*device = &dev->device.common;
	status = 0;
	return status;
}
