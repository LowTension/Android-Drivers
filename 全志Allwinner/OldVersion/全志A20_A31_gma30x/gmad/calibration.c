/*
 * CONFIDENTIAL
 * Copyright (C) 2014 Globalmems Corporation
 */

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
//#include <cutils/log.h>
//char GMA_Offset_TXT[] = "offset.txt";	/* FILE offset.txt */
//char GMA_Offset_BAK[] = "offset_bak.txt";	/* FILE offset.txt */
#define GMA_BACKUP_FILE
#ifdef GMA_BACKUP_FILE
static const char *BACKUP_FILE_PATH = "offset.txt";
static const char *TEMPORARY_BACKUP_PATH = "offset_bak.txt";
static const char *DEST_FILE_PATH="offset_dest.txt";
#endif
/*int save_offset()
{
	int buffer = 9;
	FILE *file=fopen(GMA_Offset_TXT,"wt"); // open offset
	//fwrite(offset,sizeof(int),SENSOR_DATA_SIZE,file); //二進制存檔
	fprintf(file,"%d", buffer); //格式化資料存檔
	printf("%d : %d", buffer, __LINE__);
	fclose(file);
	
 	return 0;
}*/
#ifdef GMA_BACKUP_FILE
int read_offset(const char *src,const char *dstFilePath)
{
	int i, buffer[3];
	FILE *fin=fopen(src,"r"); // open offset.txt
	FILE *fout=fopen(dstFilePath,"wt"); // save tmp
	//fread(offset, sizeof(int),SENSOR_DATA_SIZE,fin);//讀檔
	while (EOF != fscanf(fin,"%d %d %d", &buffer[0], &buffer[1], &buffer[2]))//格式化資料讀檔
		//printf("%d %d %d : %d\n", buffer[0], buffer[1], buffer[2], __LINE__);
	
	//fwrite(offset,sizeof(int),SENSOR_DATA_SIZE,file); //存檔
	fprintf(fout,"%d %d %d", buffer[0], buffer[1], buffer[2]); //格式化資料存檔
	
	fclose(fin);
	fclose(fout);
	
 	return 0;
}
#endif
int main(void)
{
#ifdef GMA_BACKUP_FILE
	read_offset(BACKUP_FILE_PATH,TEMPORARY_BACKUP_PATH);
#endif

#ifdef GMA_BACKUP_FILE
	read_offset(TEMPORARY_BACKUP_PATH,DEST_FILE_PATH);
	chmod(DEST_FILE_PATH, 0666);
	chown(DEST_FILE_PATH, 1000, 1000); // system system
#endif
	return 0;
}
