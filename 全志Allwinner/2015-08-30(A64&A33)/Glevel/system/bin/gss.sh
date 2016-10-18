#!/bin/sh
# Program:
#       Globalmems Shell Script(GSS).
# Usage: 
#		sh /system/bin/gss.sh calib asix
#		sh		   		($0)	($1)($2)
#		sh /system/bin/gss.sh clear_offset
#		sh		   		($0)	($1)
#		sh /system/bin/gss.sh get_offset
#		sh		   		($0)	($1)
#		sh /system/bin/gss.sh read_reg REG
#		sh		   		($0)	($1)	($2)
#		sh /system/bin/gss.sh write_reg REG VALUE
#		sh		   		($0)	($1)	($2) ($3)
#		sh /system/bin/gss.sh read_reg_map
#		sh		   		($0)	($1)
# Function:
# 		calib		: calibration & save file into /data/misc/gsensor_offset.txt(Driver level)
#		clear_offset: clear offset & save file into /data/misc/gsensor_offset.txt(Driver level)
#		get_offset	: get offset from /sys/class/input/inputX/offset
#		read_reg	: read value from register
#		write_reg	: write value to register
#		read_reg_map: Read all register value and archive to "/data/misc/reg_map.txt"
# Example Usage:
# 		sh /system/bin/gss.sh calib 9
# 		sh /system/bin/gss.sh clear_offset
# 		sh /system/bin/gss.sh get_offset
# 		sh /system/bin/gss.sh read_reg 0x12
# 		sh /system/bin/gss.sh write_reg 0x0f 0x00
# 		sh /system/bin/gss.sh read_reg_map
# History:
# 2014/06/06	Bruce	Shell Script Function Fusion
# 2014/07/21	Bruce	Add : support ACTIONS driver	Example Usage:
#			implement	1.calib()		sh /system/bin/gss.sh calib 1	
#					2.clear_offset()	sh /system/bin/gss.sh clear_offset
#					3.read_reg()		sh /system/bin/gss.sh read_reg 0x12
#					4.write_reg()		sh /system/bin/gss.sh write_reg 0x0f 0x00
#				notices: ACTIONS driver did not open "/data/misc" access
# 2014/08/22	Bruce	Add : calibration step inspection mechanism
# 2015/05/27	Bruce	Add : get layout & set position  Default:enable
# 2015/06/30	Bruce	Add : Read Gme601_accel/Gme605_accel Register MAP
# 2015/07/28	Bruce	Add : support MTK Driver
# 2015/07/29	Bruce	Add : 5.get_offset()	sh /system/bin/gss.sh get_offset
export PATH=/system/bin:/data/misc:$PATH

DIR_BIN="/system/bin"     						# The main program path
DIR_INPUT="/sys/class/input"					# The input path
ATTR_OFFSET="offset"							# [rw]	[txt] : show calibration offset
ATTR_RX="reg_rx"								# [rw] 	[txt] : Read from register(show value)
ATTR_TX="reg_tx"								# [rw] 	[txt] : The value currently being written to the register
ATTR_CHIPINFO="chipinfo"						# [r ]	[txt] : chip info
ATTR_POSITION="position"						# [rw]	[txt] : position info
ATTR_POSITION_ACTIONS="board_position"			# [rw]	[txt] : ACTIONS position info
ATTR_CALIB="calibration"						# [rw]	[txt] : ACC sensor calibration range(1~9)
###########only support ACTIONS##################
ATTR_OFFSET_ACTIONS="calibration_reset"			# [rw]	[txt] : Remind*(only support ACTIONS) clear calibration offset
ATTR_CALIB_VALUE_ACTIONS="calibration_value"	# [rw]	[txt] : Remind*(only support ACTIONS) show calibration offset
ATTR_CALIB_ACTIONS="calibration_run"			# [rw]	[txt] : Remind*(only support ACTIONS) GMA30x ACC Driver
###########only support MTK######################
DIR_MTK="/sys/bus/platform/drivers/gsensor"		# only support MTK platform path
#################################################
DIR_SENSOR="/data/misc"							# Private directory 
FILE_OFFSET="gsensor_offset.txt"				# The latest offset
FILE_OFFSET_LOG="gsensor_offset.log"			# calibration History
FILE_TEMP="tmp.log"								# temp file
FILE_REG_MAP="reg_map.txt"						# Save the register current value
RUN_CALIB="gmad"								# Use ioctl to calibration gsensor
NOW=`date +%Y%m%d_%H%M`							# date format Example: 20140822_1636
now=`date`										# date format Example: 五 8月 22 16:36:24 CST 2014

function cab () { 
	case $1 in 
		"9") 
		echo "step 4: automatically determine the Z direction"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"8") 
		echo "step 4: automatically determine the Y direction"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"7") 
		echo "step 4: automatically determine the X direction"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"6") 
		echo "step 4: GRAVITY_ON_X_POSITIVE"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"5") 
		echo "step 4: GRAVITY_ON_X_NEGATIVE"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"4") 
		echo "step 4: GRAVITY_ON_Y_POSITIVE"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"3") 
		echo "step 4: GRAVITY_ON_Y_NEGATIVE"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"2") 
		echo "step 4: GRAVITY_ON_Z_POSITIVE"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		"1") 
		echo "step 4: GRAVITY_ON_Z_NEGATIVE"
		echo "$1" > $DIR_INPUT/$inputpath/$ATTR_CALIB
		;;
		
		* ) 
		#echo 9 > $DIR_INPUT/$inputpath/$ATTR_CALIB
		echo "Example Usage: sh gss.sh calib 9" 
		;;
	esac
}

function cat_reg () { 
	case $1 in 
		"$1") 
		echo $1 > $DIR_INPUT/$inputpath/$ATTR_RX
		echo "REG($1) `cat $DIR_INPUT/$inputpath/$ATTR_RX`" >> $DIR_SENSOR/$FILE_REG_MAP
		#echo "REG($1) value is `cat $DIR_INPUT/$inputpath/$ATTR_RX`  ( $now)" >> $FILE_REG_MAP
		;;
		
		* ) 
		echo "Example Usage: sh gss.sh calib 9" 
		;;
	esac
}

function reg_map () { 
	case "$1" in
	"gma301") 
	echo "Read Gma301 Register MAP" > $DIR_SENSOR/$FILE_REG_MAP
	cat_reg 0x00 
	cat_reg 0x0C 
	cat_reg 0x0D 
	cat_reg 0x0E 
	cat_reg 0x0F 
	cat_reg 0x11 
	cat_reg 0x12 
	cat_reg 0x13 
	cat_reg 0x14 
	cat_reg 0x15 
	cat_reg 0x16 
	cat_reg 0x17 
	cat_reg 0x18 
	cat_reg 0x19 
	cat_reg 0x1A 
	cat_reg 0x1B 
	cat_reg 0x1C 
	cat_reg 0x1F 
	cat_reg 0x21
	;;
	
	"gma302")
	echo "Read Gma302 Register MAP" > $DIR_SENSOR/$FILE_REG_MAP
	cat_reg 0x00 
	cat_reg 0x01 
	cat_reg 0x02 
	cat_reg 0x03 
	cat_reg 0x04 
	cat_reg 0x05 
	cat_reg 0x06 
	cat_reg 0x07 
	cat_reg 0x08 
	cat_reg 0x09 
	cat_reg 0x0a 
	cat_reg 0x0b 
	cat_reg 0x0c 
	cat_reg 0x0d 
	cat_reg 0x15 
	cat_reg 0x16 
	cat_reg 0x17 
	cat_reg 0x18 
	cat_reg 0x38
	;;
	
	"gma303")
	echo "Read Gma303 Register MAP" > $DIR_SENSOR/$FILE_REG_MAP
	cat_reg 0x00 
	cat_reg 0x01 
	cat_reg 0x02 
	cat_reg 0x03 
	cat_reg 0x04 
	cat_reg 0x05 
	cat_reg 0x06 
	cat_reg 0x07 
	cat_reg 0x08 
	cat_reg 0x09 
	cat_reg 0x0a 
	cat_reg 0x0b 
	cat_reg 0x0c 
	cat_reg 0x0d 
	cat_reg 0x15 
	cat_reg 0x16 
	cat_reg 0x17 
	cat_reg 0x18 
	cat_reg 0x38
	;;
	
	"gma305")
	echo "Read Gma305 Register MAP" > $DIR_SENSOR/$FILE_REG_MAP
	cat_reg 0x00 
	cat_reg 0x01 
	cat_reg 0x02 
	cat_reg 0x03 
	cat_reg 0x04 
	cat_reg 0x05 
	cat_reg 0x06 
	cat_reg 0x07 
	cat_reg 0x08 
	cat_reg 0x09 
	cat_reg 0x0a 
	cat_reg 0x0b 
	cat_reg 0x0c 
	cat_reg 0x0d 
	cat_reg 0x15 
	cat_reg 0x16 
	cat_reg 0x17 
	cat_reg 0x18 
	cat_reg 0x38
	;;
	
	"gme601a")
	echo "Read Gme601_accel Register MAP" > $DIR_SENSOR/$FILE_REG_MAP
	cat_reg 0x00 
	cat_reg 0x01 
	cat_reg 0x02 
	cat_reg 0x03 
	cat_reg 0x04 
	cat_reg 0x05 
	cat_reg 0x06 
	cat_reg 0x07 
	cat_reg 0x08 
	cat_reg 0x09 
	cat_reg 0x0a 
	cat_reg 0x0b 
	cat_reg 0x0c 
	cat_reg 0x0d 
	cat_reg 0x15 
	cat_reg 0x16 
	cat_reg 0x17 
	cat_reg 0x18 
	cat_reg 0x38
	;;
	
	"gme605a")
	echo "Read Gme605_accel Register MAP" > $DIR_SENSOR/$FILE_REG_MAP
	cat_reg 0x00 
	cat_reg 0x01 
	cat_reg 0x02 
	cat_reg 0x03 
	cat_reg 0x04 
	cat_reg 0x05 
	cat_reg 0x06 
	cat_reg 0x07 
	cat_reg 0x08 
	cat_reg 0x09 
	cat_reg 0x0a 
	cat_reg 0x0b 
	cat_reg 0x0c 
	cat_reg 0x0d 
	cat_reg 0x15 
	cat_reg 0x16 
	cat_reg 0x17 
	cat_reg 0x18 
	cat_reg 0x38
	;;
	
    * ) 
	echo "default nothing"
	echo "Example Usage: sh gss.sh read_reg_map" 
	;;
esac
}

save_offset(){
	# confirm $FILE_OFFSET_LOG exists & save offset
	NEW_offset=`cat $DIR_INPUT/$inputpath/offset`
	#boot_times=`cat $DIR_INPUT/$inputpath/boottimes`
	if [ -f "$DIR_SENSOR/$FILE_OFFSET" ]; then
		echo "step 5: File ${FILE_OFFSET} exists. \n\tRead old offset $OLD_offset.\n\tSave new offset $NEW_offset."
		cp $DIR_SENSOR/$FILE_OFFSET_LOG $DIR_SENSOR/$FILE_TEMP
		#rm $FILE_OFFSET_LOG
		echo "$NEW_offset $now" > $DIR_SENSOR/$FILE_OFFSET_LOG
		cat $DIR_SENSOR/$FILE_TEMP >> $DIR_SENSOR/$FILE_OFFSET_LOG
		rm $DIR_SENSOR/$FILE_TEMP
		echo "$NEW_offset" > $DIR_SENSOR/$FILE_OFFSET
		#echo "$NEW_offset" "$boot_times" > $DIR_SENSOR/$FILE_OFFSET
	else
		echo "$NEW_offset $now" >> $DIR_SENSOR/$FILE_OFFSET_LOG
		echo "$NEW_offset" > $DIR_SENSOR/$FILE_OFFSET
		#echo "$NEW_offset" "$boot_times" > $DIR_SENSOR/$FILE_OFFSET
	fi
}

# Main
if [ ! -d ${DIR_SENSOR} ] ; then
	mkdir ${DIR_SENSOR}
	echo "The folder not existed. Now create ${DIR_SENSOR}"
fi
#cd ${DIR_SENSOR}

# $inputpath : scan ACC driver registered in the input?
#	1.GMA driver	: scan $inputpath/calibration
#	2.ACTION driver	: scan $inputpath/calibration_run
cd $DIR_INPUT
for inputpath in input*
do
	if [ -w "$inputpath/$ATTR_CALIB" ]; then
		var=$DIR_INPUT/$inputpath/$ATTR_CALIB
		#echo step 1: The file path was ${var%%.*}
		INTERACTIVE=0 # calibration go path 1
		break;
	elif [ -w "$inputpath/$ATTR_CALIB_ACTIONS" ]; then
		var=$DIR_INPUT/$inputpath/$ATTR_CALIB
		#echo step 1: The file path was ${var%%.*}
		INTERACTIVE=0 # calibration go path 1
		break;
	else
		#echo "ERR step 1: did not find $inputpath/$ATTR_CALIB"
		inputpath=NULL
		INTERACTIVE=1 # calibration go path 2
	fi
done

if [ -x $DIR_BIN/$RUN_CALIB -a $INTERACTIVE -eq 1 ] ; then
	case "$1" in
		"calib")
		if $DIR_BIN/$RUN_CALIB -c $2 ; then
			echo "The second path calibration method. Run gmad"
		else
			echo "Run gmad fail. Please check /system/bin/gmad"
		fi
		exit 1
		;;
		
		"clear_offset")

		;;
		
		* ) 	
		;;
	esac
#else
#	echo "Run gmad fail. Please check /system/bin/gmad Permissions"
fi

#######only support MTK Driver################
#check -w /sys/bus/platform/drivers/gsensor/calibration
#check INTERACTIVE =1
if [ -w $DIR_MTK/$ATTR_CALIB -a $INTERACTIVE -eq 1 ] ; then
	case "$1" in
	"calib")
	echo "step 1: Start MTK calibration.  ( $now)"
		case "$2" in          # check radio button
		  "1")
			if [ -f "$DIR_MTK/$ATTR_CALIB" ]; then
				echo 0 0 0 > $DIR_MTK/$ATTR_OFFSET
				sleep 1
				echo $2 > $DIR_MTK/$ATTR_CALIB
				cat $DIR_MTK/$ATTR_OFFSET
			else
				echo did not find $DIR_MTK/$ATTR_CALIB
				exit 1
			fi
			;;
		  "2")
			if [ -f "$DIR_MTK/$ATTR_CALIB" ]; then
				echo 0 0 0 > $DIR_MTK/$ATTR_OFFSET
				sleep 1
				echo $2 > $DIR_MTK/$ATTR_CALIB
				cat $DIR_MTK/$ATTR_OFFSET
			else
				echo did not find $DIR_MTK/$ATTR_CALIB
				exit 1
			fi
			;;
		  "9")
		  	if [ -f "$DIR_MTK/$ATTR_POSITION" ]; then
				echo 0 0 0 > $DIR_MTK/$ATTR_OFFSET
				sleep 1
				layout=`cat $DIR_MTK/$ATTR_POSITION`
				if [ "$layout" -lt 0 ] ; then
					echo 1 > $DIR_MTK/$ATTR_CALIB
					cat $DIR_MTK/$ATTR_OFFSET
					#echo "388 Get layout= " $layout " Set $2  ( $now)"
				else
					echo 2 > $DIR_MTK/$ATTR_CALIB
					cat $DIR_MTK/$ATTR_OFFSET
					#echo "392 Get layout= " $layout " Set $2  ( $now)"
				fi
				echo "Get layout= " $layout " Set $2  ( $now)"
			#else
			#	echo "did not find $inputpath/$ATTR_POSITION"
			fi
			;;
		  * )
			echo "$2 not accept"
			;;
			
		esac	
	;;
	"clear_offset")
	if [ -f "$DIR_MTK/$ATTR_OFFSET" ]; then
		echo 0 0 0 > $DIR_MTK/$ATTR_OFFSET
		echo "clear offset `cat $DIR_MTK/$ATTR_OFFSET`"
	else
		exit 1
	fi
	
	NEW_MTKoffset=`cat $DIR_MTK/$ATTR_OFFSET`
	if [ -f "$DIR_SENSOR/$FILE_OFFSET" ]; then
		echo "step 2: File ${FILE_OFFSET} exists. \n\tRead old offset $OLD_offset.\n\tSave new offset $NEW_MTKoffset."
		cp $DIR_SENSOR/$FILE_OFFSET_LOG $DIR_SENSOR/$FILE_TEMP
		#rm $FILE_OFFSET_LOG
		#echo "$NEW_MTKoffset $now" > $DIR_SENSOR/$FILE_OFFSET_LOG
		#cat $DIR_SENSOR/$FILE_TEMP >> $DIR_SENSOR/$FILE_OFFSET_LOG
		rm $DIR_SENSOR/$FILE_TEMP
		#echo "$NEW_MTKoffset" > $DIR_SENSOR/$FILE_OFFSET
	else
		echo "step 2: File ${FILE_OFFSET} no exist."
		#echo "$NEW_MTKoffset $now" >> $DIR_SENSOR/$FILE_OFFSET_LOG
		#echo "$NEW_MTKoffset" > $DIR_SENSOR/$FILE_OFFSET
	fi
	;;
	
	"get_offset")
	if [ -f "$DIR_MTK/$ATTR_OFFSET" ]; then
		NEW_MTKoffset=`cat $DIR_MTK/$ATTR_OFFSET`
		echo "$NEW_MTKoffset"
	else
		exit 1
	fi
	;;
	
    * ) 
	echo "Example Usage: "
	echo "sh /system/bin/gss.sh calib asix"
	echo "sh /system/bin/gss.sh clear_offset"
	echo "sh /system/bin/gss.sh get_offset"
	;;
	esac

	exit 0
fi
######################################################
case "$1" in			# check command
	"calib")
	echo "step 2: Start calibration.  ( $now)"
	case "$2" in          # check radio button
	  "1")
		if [ -f "$inputpath/$ATTR_CALIB" ]; then
			#OLD_offset=`cat $DIR_SENSOR/$FILE_OFFSET`
			cab $2	#use radio button position(1 or 2)
			#save_offset
		elif [ -f "$inputpath/$ATTR_CALIB_ACTIONS" ]; then
			echo 1 > $DIR_INPUT/$inputpath/$ATTR_CALIB_ACTIONS
			cat $DIR_INPUT/$inputpath/$ATTR_CALIB_VALUE_ACTIONS
		else
			echo did not find $inputpath/$ATTR_CALIB
			echo did not find $inputpath/$ATTR_CALIB_ACTIONS
			inputpath=NULL
			exit 1
		fi
		;;
	  "2")
		if [ -f "$inputpath/$ATTR_CALIB" ]; then
			#OLD_offset=`cat $DIR_SENSOR/$FILE_OFFSET`
			cab $2	#use radio button position(1 or 2)
			#save_offset
		elif [ -f "$inputpath/$ATTR_CALIB_ACTIONS" ]; then
			echo 1 > $DIR_INPUT/$inputpath/$ATTR_CALIB_ACTIONS
			cat $DIR_INPUT/$inputpath/$ATTR_CALIB_VALUE_ACTIONS
		else
			echo did not find $inputpath/$ATTR_CALIB
			echo did not find $inputpath/$ATTR_CALIB_ACTIONS
			inputpath=NULL
			exit 1
		fi
		;;
	  "9")
	  	if [ -f "$inputpath/$ATTR_POSITION" ]; then
			layout=`cat $DIR_INPUT/$inputpath/$ATTR_POSITION`
			if [ "$layout" -lt 0 ] ; then
				cab 1
			else
				cab 2
			fi
			echo "Get layout= " $layout " Set $2  ( $now)"
		else
			echo "did not find $inputpath/$ATTR_POSITION"
		fi
		;;
	  *)
		echo "$2 not accept"
		;;
	esac
	;;
	
	"clear_offset")
	if [ -f "$DIR_INPUT/$inputpath/$ATTR_OFFSET" ]; then
		echo 0 0 0 > $DIR_INPUT/$inputpath/$ATTR_OFFSET
		echo "clear offset `cat $DIR_INPUT/$inputpath/$ATTR_OFFSET`"
		#save_offset
	elif [ -f "$DIR_INPUT/$inputpath/$ATTR_OFFSET_ACTIONS" ]; then
		echo 1 > $DIR_INPUT/$inputpath/$ATTR_OFFSET_ACTIONS
		cat $DIR_INPUT/$inputpath/$ATTR_CALIB_VALUE_ACTIONS
	else
		exit 1
	fi
	;;
	
	"get_offset")
	if [ -f "$DIR_INPUT/$inputpath/$ATTR_OFFSET" ]; then
		echo "`cat $DIR_INPUT/$inputpath/$ATTR_OFFSET`"
	elif [ -f "$DIR_INPUT/$inputpath/$ATTR_OFFSET_ACTIONS" ]; then
		echo "`cat $DIR_INPUT/$inputpath/$ATTR_CALIB_VALUE_ACTIONS`"
	else
		exit 1
	fi
	;;
	
	"read_reg")
	if [ -r $DIR_INPUT/$inputpath/$ATTR_RX ]; then
		echo "$2" > $DIR_INPUT/$inputpath/$ATTR_RX
		echo "`cat $DIR_INPUT/$inputpath/$ATTR_RX`  ( $now)"
	else
		echo can not read $DIR_INPUT/$inputpath/$ATTR_RX
		exit 1
	fi
	;;
	
	"write_reg")
	if [ -w $DIR_INPUT/$inputpath/$ATTR_TX ]; then
		echo "$2" > $DIR_INPUT/$inputpath/$ATTR_RX #ADDR set reg
		echo "$3" > $DIR_INPUT/$inputpath/$ATTR_TX #write value
		echo "`cat $DIR_INPUT/$inputpath/$ATTR_TX`  ( $now)"
	else
		echo can not read $DIR_INPUT/$inputpath/$ATTR_TX
		exit 1
	fi
	;;
	
	"read_reg_map")
	if [ -r $DIR_INPUT/$inputpath/$ATTR_CHIPINFO ]; then
		chip=`cat $DIR_INPUT/$inputpath/$ATTR_CHIPINFO`
		reg_map $chip
		echo $chip Register map save to $DIR_SENSOR/$FILE_REG_MAP
	else
		echo can not read $DIR_INPUT/$inputpath/$ATTR_CHIPINFO
		exit 1
	fi
	;;
	
    * ) 
	echo "Example Usage: "
	echo "sh /system/bin/gss.sh calib asix"
	echo "sh /system/bin/gss.sh clear_offset"
	echo "sh /system/bin/gss.sh get_offset"
	echo "sh /system/bin/gss.sh read_reg REG"
	echo "sh /system/bin/gss.sh write_reg REG VALUE"
	echo "sh /system/bin/gss.sh read_reg_map"	
	;;
esac

exit 0
