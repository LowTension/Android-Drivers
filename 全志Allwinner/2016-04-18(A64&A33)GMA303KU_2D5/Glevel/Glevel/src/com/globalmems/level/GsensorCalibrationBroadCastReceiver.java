package com.globalmems.level;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
//import java.io.FileNotFoundException;
//import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
//import android.widget.Toast;

public class GsensorCalibrationBroadCastReceiver extends BroadcastReceiver {
	private static final String TAG = "BootBroadcastReceiver";
	File offset = new File("/data/misc/gsensor_offset.txt");
	boolean GMA30x_found=false,OffsetFILE_exist=false;
    String line = "";
	FileInputStream input = null;
	int[] gsensor_offset = new int[3];
	@Override
	public void onReceive(Context context, Intent intent) {
		// TODO Auto-generated method stub
		if (intent.getAction().equals(Intent.ACTION_BOOT_COMPLETED)) {
			Log.d(TAG, "ACTION_BOOT_COMPLETED!!!");
			
			if (!offset.exists()) {
				//FileInputStream fis = null;
				Runtime rt;
                rt = Runtime.getRuntime();
                String cmd = "";
                if (GMA30x_found == false ) {
                    cmd = "cat /sys/bus/platform/drivers/gsensor/chipinfo";
                    Log.d(TAG, cmd );
                    Process proc = null;
                    try {
                        proc = rt.exec(cmd.toString());
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    InputStream is = proc.getInputStream();
                    InputStreamReader isr = new InputStreamReader(is);
                    BufferedReader br = new BufferedReader(isr);
                    String GMA302="gma302";
                    String GMA303="GMA30x Chip";
                    String GMA30x="gma30x";
                    String GMA333="gma333";
                    String GMA305="gma305";
                    String GME605="gme605_accel";
                    String GME601="gme601_accel";
                    try {
                        while ((line = br.readLine()) != null) {
                        	Log.d(TAG, line );
                            if (line.equals(GMA302) || line.equals(GMA303) || line.equals(GMA30x) || line.equals(GMA333) || line.equals(GMA305)|| line.equals(GME605)|| line.equals(GME601)) {
                                GMA30x_found = true;
                                Log.d(TAG, "Broadcast: found gsensor " + line);
                                break;
                            }
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                if(GMA30x_found) {
                                    Intent nintent = new Intent(context, Level.class);
                                    nintent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
                                    context.startActivity(nintent);
                                    Log.d(TAG, "offset does not exist , GMA30x found");
/*                	Runtime rt1;
      	            rt1 = Runtime.getRuntime();
                	cmd="sh /system/bin/gss.sh calib 9";
                    try {
			        Process proc =rt1.exec(cmd.toString());
	                
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
*/                }
			}else{
				String cmd = "cat /data/misc/gsensor_offset.txt";
                Process proc = null;
                Runtime rtime;
                rtime = Runtime.getRuntime();
                try {
                    proc = rtime.exec(cmd.toString());
                    InputStream is = proc.getInputStream();
                    InputStreamReader isr = new InputStreamReader(is);
                    BufferedReader br = new BufferedReader(isr);             
                        while ((line = br.readLine()) != null) {
                        	System.out.println(line); /* For debug use*/
                            String[] result = line.split("\\s");
                            if (result[0].equals("0") && result[1].equals("0") && result[2].equals("0")) {
                                Intent nintent = new Intent(context, Level.class);
                                nintent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
                                context.startActivity(nintent);
                                Log.d(TAG, "offset exist , GMA30x found");
                                break;
                            }
                        }                 
                } catch (IOException e) {
                    e.printStackTrace();
				}
			}
		}
	}
	
}