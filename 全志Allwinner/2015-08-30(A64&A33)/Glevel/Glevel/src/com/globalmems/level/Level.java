package com.globalmems.level;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.text.DecimalFormat;

import com.globalmems.level.config.Provider;
import com.globalmems.level.orientation.Orientation;
import com.globalmems.level.orientation.OrientationListener;
import com.globalmems.level.orientation.OrientationProvider;
import com.globalmems.level.view.LevelView;

import com.globalmems.level.R;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.hardware.SensorManager;
import android.media.AudioManager;
import android.media.SoundPool;
import android.os.Bundle;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import android.os.SystemClock;
import android.preference.PreferenceManager;
import android.util.FloatMath;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.RadioGroup;
import android.widget.RadioButton;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.preference.Preference;
import android.preference.PreferenceGroup;
/*
 *  This file is part of Level (an Android Bubble Level).
 *  <https://github.com/avianey/Level>
 *  
 *  Copyright (C) 2012 Antoine Vianey
 *  
 *  Level is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Level is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Level. If not, see <http://www.gnu.org/licenses/>
 */
public class Level extends Activity implements OrientationListener {
	private static final String TAG = "Level";
	private static Level CONTEXT;
	
	private static final int DIALOG_CALIBRATE_ID = 1;
	private static final int TOAST_DURATION = 10000;
	
	private OrientationProvider provider;
	
    private LevelView view;
    private WakeLock wakeLock;
    
	/** Gestion du son */
	private SoundPool soundPool;
	private boolean soundEnabled;
	private int bipSoundID;
	private int bipRate;
	private long lastBip;
	
	private Button mClose;
	private CheckBox mShowDetail;
	private LinearLayout mLayout;
	/* For Gsensor Calibration*/
	String str[] = { "Fastest", "Game  ", "UI     ", "Normal " };
	public  int fd = 0;
	int[] xyz = new int[3];
	private int[] z_direction = new int[3];
	DecimalFormat of = new DecimalFormat("  #000;-#000");
	DecimalFormat nf = new DecimalFormat("  #0.0000;-#00.0000  ");
    DecimalFormat tds = new DecimalFormat(" #,###,000");
	private Button cabiration1;
	private Button clear_cab;
	private Button close_device;
	private Button delay;
	private TextView accelerometer_x;
	private TextView accelerometer_y;
	private TextView accelerometer_z;
	private TextView acc_offset_x;
	private TextView acc_offset_y;
	private TextView acc_offset_z;
	private TextView acc_sigma_x;
	private TextView acc_sigma_y;
	private TextView acc_sigma_z;
	//private TextView magnetic;
	//private TextView magnetic_x;
	//private TextView magnetic_y;
	//private TextView magnetic_z;
	//private TextView mag_sigma_x;
	//private TextView mag_sigma_y;
	//private TextView mag_sigma_z;
	//private TextView orientation;
	//private TextView orientation_x;
	//private TextView orientation_y;
	//private TextView orientation_z;
	//private TextView ori_sigma_x;
	//private TextView ori_sigma_y;
	//private TextView ori_sigma_z;
	private SensorManager mSensorManager01;
	int delay_mode=2;
	long TimeNewACC;
	long delayACC;
	static long TimeOldACC;
	  
	float[] sum_acc_XYZ = new float[3];
	float[] sumAccSquare= new float[3];
	float[] sigma_acc=new float[3];
	int countS_acc=0;
	float[] sum_mag_XYZ = new float[3];
	float[] sumMagSquare= new float[3];
	float[] sigma_mag=new float[3];
	int countS_mag=0;
	float[] sum_ori_XYZ = new float[3];
	float[] sumOriSquare= new float[3];
	float[] sigma_ori=new float[3];
	int countS_ori=0;
	
	int calibNUM=1;
	String cmd=new String();
	FileInputStream input = null;
	private RadioGroup RdGpCalib;
    private RadioButton rdbUp,rdbDown,rdbDefault;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        //addShortcut();
 /*        
        Button add = (Button) findViewById(R.id.bAdd_Shortcut);
        add.setOnClickListener(new OnClickListener() {
             @Override
            public void onClick(View v) {
                addShortcutIcon(); 
            }
        });
        Button remove = (Button) findViewById(R.id.bRemove);
        remove.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                removeShortcutIcon(); 
            }
        });     
 */       CONTEXT = this;
        view = (LevelView) findViewById(R.id.level);
        // sound
    	soundPool = new SoundPool(1, AudioManager.STREAM_RING, 0);
    	bipSoundID = soundPool.load(this, R.raw.bip, 1);
    	bipRate = getResources().getInteger(R.integer.bip_rate);
    	
    	/* ��?����?�SensorManager */
        mSensorManager01 =(SensorManager)getSystemService(Context.SENSOR_SERVICE);
        accelerometer_x = (TextView) findViewById(R.id.accelerometer_x);
        accelerometer_y = (TextView) findViewById(R.id.accelerometer_y);
        accelerometer_z = (TextView) findViewById(R.id.accelerometer_z);
        acc_offset_x = (TextView) findViewById(R.id.acc_offset_x);
        acc_offset_y = (TextView) findViewById(R.id.acc_offset_y);
        acc_offset_z = (TextView) findViewById(R.id.acc_offset_z);
        acc_sigma_x = (TextView) findViewById(R.id.acc_sigma_x);
        acc_sigma_y = (TextView) findViewById(R.id.acc_sigma_y);
        acc_sigma_z = (TextView) findViewById(R.id.acc_sigma_z);
        z_direction[0] = 2;
        accelerometer_x.setText("  X : " + "123455656" );
        accelerometer_y.setText("  Y : " + "123455656" );
        accelerometer_z.setText("  Z : " + "123455656" );
        cabiration1 =(Button) findViewById(R.id.cab_1);
        clear_cab =(Button) findViewById(R.id.clear_cab);
        close_device = (Button) findViewById(R.id.close);
		delay = (Button) findViewById(R.id.delay);
        cabiration1.setOnClickListener(cab1_listener);
        clear_cab.setOnClickListener(clear_cab_listener);
        close_device.setOnClickListener(close_listener);
		delay.setOnClickListener(delay_listener);
		
		rdbUp=(RadioButton)findViewById(R.id.rdbUp);
        rdbDown=(RadioButton)findViewById(R.id.rdbDown);
        rdbDefault=(RadioButton)findViewById(R.id.rdbDefault);
        RdGpCalib=(RadioGroup)findViewById(R.id.RdGpCalib);
        RdGpCalib.setOnCheckedChangeListener(CalibOption);
		int[] result = new int[3];
        /* radio button default use rdbUp/rdbDown/rdbDefault */
        //rdbUp.setChecked(true);

		 acc_offset_x.setText("  X : " + of.format(result[0]) + "  ");
		 acc_offset_y.setText("  Y : " + of.format(result[1]) + "  ");
		 acc_offset_z.setText("  Z : " + of.format(result[2]) + "  ");
		 //Log.e(TAG, "acc_offset X/Y/Z: " + acc_offset_x + " / " + acc_offset_y + " / " + acc_offset_z );
        mSensorManager01.registerListener 
        ( 
          mSensorListener, 
          mSensorManager01.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), bipRate, null
        );
        
        //mClose = (Button) findViewById(R.id.cab_cancel);
		mLayout = (LinearLayout) findViewById(R.id.detail_layout);
		mShowDetail = (CheckBox) findViewById(R.id.show_detail);
		//mClose.setOnClickListener(close_listener);
		if (mShowDetail.isChecked()) {
			mLayout.setVisibility(View.VISIBLE);
		} else {
			mLayout.setVisibility(View.GONE);
		}
		mShowDetail.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

					@Override
					public void onCheckedChanged(CompoundButton buttonView,
							boolean isChecked) {
						if (isChecked) {
							mLayout.setVisibility(View.VISIBLE);
						} else {
							mLayout.setVisibility(View.GONE);
						}
					}
				});
		//if(rdbDefault.isChecked()==false)
		//rdbDefault.setChecked(true);
        rdbDown.setChecked(true);
		//rdbDefault.callOnClick();
    }
    
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
	    MenuInflater inflater = getMenuInflater();
	    inflater.inflate(R.menu.main, menu);
	    return true;
	}
    
	public void addShortcut() {  
		 Intent shortcutIntent = new Intent(getApplicationContext(),  
				 Level.class); // 啟動捷徑入口，一般用MainActivity，有使用其他入口則填入相對名稱，ex:有使用SplashScreen  
		 shortcutIntent.setAction(Intent.ACTION_MAIN);  
		 Intent addIntent = new Intent();  
		 addIntent.putExtra(Intent.EXTRA_SHORTCUT_INTENT, shortcutIntent); // shortcutIntent送入  
		 addIntent.putExtra(Intent.EXTRA_SHORTCUT_NAME,  
		   getString(R.string.name)); // 捷徑app名稱  
		 addIntent.putExtra(Intent.EXTRA_SHORTCUT_ICON_RESOURCE,  
		   Intent.ShortcutIconResource.fromContext(  
		     getApplicationContext(),// 捷徑app圖  
		     R.drawable.icon));  
		 addIntent.putExtra("duplicate", false); // 只創建一次  
		 addIntent.setAction("com.android.launcher.action.INSTALL_SHORTCUT"); // 安裝  
		 getApplicationContext().sendBroadcast(addIntent); // 送出廣播       
		}
	
	// onClick of addShortcutIcon 
    private void addShortcutIcon() {
        //shorcutIntent object
        Intent shortcutIntent = new Intent(getApplicationContext(),
                Level.class);
        
        shortcutIntent.setAction(Intent.ACTION_MAIN);
        //shortcutIntent is added with addIntent
        Intent addIntent = new Intent();
        addIntent.putExtra(Intent.EXTRA_SHORTCUT_INTENT, shortcutIntent);
        addIntent.putExtra(Intent.EXTRA_SHORTCUT_NAME,getString(R.string.name));
        addIntent.putExtra("duplicate", false);
        // Set the custom shortcut icon
        addIntent.putExtra(Intent.EXTRA_SHORTCUT_ICON_RESOURCE, Intent.ShortcutIconResource.fromContext(this, R.drawable.icon));
        addIntent.setAction("com.android.launcher.action.INSTALL_SHORTCUT"); 
        // finally broadcast the new Intent
        getApplicationContext().sendBroadcast(addIntent);
    }
     
    private void removeShortcutIcon() {
         
        Intent shortcutIntent = new Intent(getApplicationContext(),
                Level.class);
        shortcutIntent.setAction(Intent.ACTION_MAIN);
         
        Intent addIntent = new Intent();
        addIntent.putExtra(Intent.EXTRA_SHORTCUT_INTENT, shortcutIntent);
        addIntent.putExtra(Intent.EXTRA_SHORTCUT_NAME, "Icon");

        addIntent.setAction("com.android.launcher.action.UNINSTALL_SHORTCUT");
        getApplicationContext().sendBroadcast(addIntent);
    }
	// Radio: calibNUM= Back(1) or Front(2) 
	private RadioGroup.OnCheckedChangeListener CalibOption=new RadioGroup.OnCheckedChangeListener() {
	        @Override
	        public void onCheckedChanged(RadioGroup radioGroup, int i) {
	            switch (i)
	            {
	                case R.id.rdbUp:
	                    calibNUM=1;
	                    break;
	                case R.id.rdbDown:
	                    calibNUM=2;
	                    break;
	                case R.id.rdbDefault:
	                    calibNUM=9;
	                    break;
	            }
	           // cabiration1.setText(String.format("Calibration %d",calibNUM));
	        }
	    };
	private Button.OnClickListener cab1_listener= new Button.OnClickListener(){
		  public void onClick(View v)
    {		 
			  Runtime rt;
	          rt = Runtime.getRuntime();
	          Log.d(TAG, "cab1_listener");
			  try {
				  cmd="sh /system/bin/gss.sh calib ";
			        cmd+=String.format("%d",calibNUM);
			        Process proc =rt.exec(cmd.toString());
			        Log.d(TAG, cmd);
			        SystemClock.sleep(200);/* waiting write gsensor_offset.txt*/
			        String line;
			        cmd = "cat /data/misc/gsensor_offset.txt";
			        Log.d(TAG, cmd);
                    Process proc1 = null;
                    try {
                        proc1 = rt.exec(cmd.toString());
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                  InputStream is = proc1.getInputStream();
                    InputStreamReader isr = new InputStreamReader(is);
                     BufferedReader br = new BufferedReader(isr);
                    br = new BufferedReader(isr);
                  if((line = br.readLine()) != null) {
                     System.out.println(line); // For debug use
                      //acc_offset_x.setText(line);
                      String[] result = line.split("\\s");

                      acc_offset_x.setText("  X : " + result[0] + "  ");
             		  acc_offset_y.setText("  Y : " + result[1] + "  ");
             		  acc_offset_z.setText("  Z : " + result[2] + "  ");
                    }
                    //Log.d(TAG, String.format("************* %f %f %f\n", result[0], result[1], result[2]));
				} catch (IOException e) {
	// TODO Auto-generated catch block
	e.printStackTrace();
}finally {
				}  	  
    	}
	};
	private Button.OnClickListener clear_cab_listener= new Button.OnClickListener(){
		  public void onClick(View v)
  {		 
			  Runtime rt;
	          rt = Runtime.getRuntime();
			  try {
				  cmd="sh /system/bin/gss.sh clear_offset ";
			        cmd+=String.format("%d",calibNUM);
			        //Log.d("EmSensor", String.format("%s\n", cmd));
			        Process proc =rt.exec(cmd.toString());
			        SystemClock.sleep(200); /* waiting write gsensor_offset.txt*/
			        String line;
			        cmd = "cat /data/misc/gsensor_offset.txt";
                    Process proc1 = null;
                    try {
                        proc1 = rt.exec(cmd.toString());
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    InputStream is = proc1.getInputStream();
                    InputStreamReader isr = new InputStreamReader(is);
                    BufferedReader br = new BufferedReader(isr);
                    br = new BufferedReader(isr);
                    if((line = br.readLine()) != null) {
                     System.out.println(line);
                      String[] result = line.split("\\s");
                     acc_offset_x.setText("  X : " + Float.parseFloat(result[0]) + "  ");
  		 			 acc_offset_y.setText("  Y : " + Float.parseFloat(result[1]) + "  ");
  		 			 acc_offset_z.setText("  Z : " + Float.parseFloat(result[2]) + "  ");
                    }
					// float[] result = new float[3];
		 			System.out.println("Level : clear_cab_listener");
		 			 //Log.d("EmSensor", String.format("************* %f %f %f\n", result[0], result[1], result[2]));

				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}finally {
					//Log.d(TAG, "do nothing");
				}  	  
  	}
	};
	private Button.OnClickListener close_listener = new Button.OnClickListener() {
		public void onClick(View v) {
			//Linuxc.close();
			finish();
		}
	};
	private Button.OnClickListener delay_listener = new Button.OnClickListener() {

		public void onClick(View v) {
			// TODO Auto-generated method stub
			//delay_mode++;
			//delay_mode %= 4;

			mSensorManager01.registerListener(mSensorListener, mSensorManager01
					.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), delay_mode);
			delay.setText("Delay mode=" + str[0]);

		}

	};

	final SensorEventListener mSensorListener = new SensorEventListener()
    {
      private float[] mGravity = new float[3];
      
      @Override
      public void onAccuracyChanged(Sensor sensor, int accuracy)
      {
        // TODO Auto-generated method stub
      }

      @Override
      public void onSensorChanged(SensorEvent event)
      {
		// TODO Auto-generated method stub
          DecimalFormat tds = new DecimalFormat(" #,###,000");
       switch (event.sensor.getType())
        {
          case Sensor.TYPE_ACCELEROMETER:
            System.arraycopy(event.values, 0, mGravity, 0, 3);
            accelerometer_x.setText("  X : " + nf.format(mGravity[0]) );
            accelerometer_y.setText("  Y : " + nf.format(mGravity[1]) );
            accelerometer_z.setText("  Z : " + nf.format(mGravity[2]) );
            TimeNewACC = event.timestamp;
            delayACC = (long)((TimeNewACC - TimeOldACC)/1000000);//
            //Log.d("@@@Sensor.TYPE_ACCELEROMETER@@@", delayACC + " ms");
            delay.setText("Delay mode="+str[0]+tds.format(delayACC)+" ms");
            TimeOldACC = TimeNewACC;
            
            sum_acc_XYZ[0] +=mGravity[0];
            sum_acc_XYZ[1] +=mGravity[1];
            sum_acc_XYZ[2] +=mGravity[2];
            sumAccSquare[0]+=mGravity[0]*mGravity[0];
            sumAccSquare[1]+=mGravity[1]*mGravity[1];
            sumAccSquare[2]+=mGravity[2]*mGravity[2];
            countS_acc++;
            if(countS_acc==100)
            {
            	sum_acc_XYZ[0]/=100;
            	sum_acc_XYZ[1]/=100;
            	sum_acc_XYZ[2]/=100;
             	sigma_acc[0]=FloatMath.sqrt(sumAccSquare[0]/100-sum_acc_XYZ[0]*sum_acc_XYZ[0]);
            	sigma_acc[1]=FloatMath.sqrt(sumAccSquare[1]/100-sum_acc_XYZ[1]*sum_acc_XYZ[1]);
            	sigma_acc[2]=FloatMath.sqrt(sumAccSquare[2]/100-sum_acc_XYZ[2]*sum_acc_XYZ[2]);
            	sum_acc_XYZ[0] =0;
            	sum_acc_XYZ[1] =0;
            	sum_acc_XYZ[2] =0;
                sumAccSquare[0]=0;
                sumAccSquare[1]=0;
                sumAccSquare[2]=0;
            	countS_acc=0;            	
            	
            	acc_sigma_x.setText("  X : " + nf.format(sigma_acc[0]) );
                acc_sigma_y.setText("  Y : " + nf.format(sigma_acc[1]) );
                acc_sigma_z.setText("  Z : " + nf.format(sigma_acc[2]) );
            }
          break;
          case Sensor.TYPE_MAGNETIC_FIELD:
            break;
          default:
            return;
        }
      }
    };
    
    /* Handles item selections */
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
	        case R.id.calibrate:
	            showDialog(DIALOG_CALIBRATE_ID);
	            return true;
	        case R.id.preferences:
	            startActivity(new Intent(this, LevelPreferences.class));
	            return true;
        }
        return false;
    }
    
    protected void unregisterReceiver(OnClickListener onClickListener) {
		// TODO Auto-generated method stub
		
	}

	protected Dialog onCreateDialog(int id) {
        Dialog dialog;
        switch(id) {
	        case DIALOG_CALIBRATE_ID:
	        	AlertDialog.Builder builder = new AlertDialog.Builder(this);
	        	builder.setTitle(R.string.calibrate_title)
	        			.setIcon(null)
	        			.setCancelable(true)
	        			.setPositiveButton(R.string.calibrate, new DialogInterface.OnClickListener() {
	        	           	public void onClick(DialogInterface dialog, int id) {

	        	           	}
	        			})
	        	       	.setNegativeButton(R.string.cancel, null)
	        	       	.setNeutralButton(R.string.reset, new DialogInterface.OnClickListener() {
	        	           	public void onClick(DialogInterface dialog, int id) {

	        	           	}
	        	       	})
	        	       	.setMessage(R.string.calibrate_message);
	        	dialog = builder.create();
	            break;
	        default:
	            dialog = null;
        }
        return dialog;
    }
    
    @SuppressWarnings("deprecation")
	protected void onResume() {
    	super.onResume();
    	mSensorManager01.registerListener 
        ( 
         mSensorListener, 
         mSensorManager01.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
         delay_mode
        );
    	//Log.d("Level", "Level resumed");
    	SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
    	provider = Provider.valueOf(
    			prefs.getString(LevelPreferences.KEY_SENSOR, 
    					LevelPreferences.PROVIDER_ACCELEROMETER)).getProvider();
    	// chargement des effets sonores
        soundEnabled = prefs.getBoolean(LevelPreferences.KEY_SOUND, false);
        // orientation manager
        if (provider.isSupported()) {
    		provider.startListening(this);
    	} else {
    		Toast.makeText(this, getText(R.string.not_supported), TOAST_DURATION).show();
    	}
        // wake lock
        wakeLock = ((PowerManager) getSystemService(Context.POWER_SERVICE)).newWakeLock(
        		PowerManager.SCREEN_BRIGHT_WAKE_LOCK, this.getClass().getName());
        wakeLock.acquire();
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (provider.isListening()) {
        	provider.stopListening();
    	}
        mSensorManager01.unregisterListener(mSensorListener);
		wakeLock.release();
    }
    
    @Override
    public void onDestroy() {
		if (soundPool != null) {
			soundPool.release();
		}
		super.onDestroy();
    }
    @Override
	protected void onStop() {
		// TODO Auto-generated method stub
		super.onStop();
	}
	@Override
	public void onOrientationChanged(Orientation orientation, float pitch, float roll) {
		view.onOrientationChanged(orientation, pitch, roll);
	}

	@Override
	public void onCalibrationReset(boolean success) {
		Toast.makeText(this, success ? 
				R.string.calibrate_restored : R.string.calibrate_failed, 
				Level.TOAST_DURATION).show();
	}

	@Override
	public void onCalibrationSaved(boolean success) {
		Toast.makeText(this, success ? 
				R.string.calibrate_saved : R.string.calibrate_failed,
				Level.TOAST_DURATION).show();
	}

    public static Level getContext() {
		return CONTEXT;
	}
    
    public static OrientationProvider getProvider() {
    	return getContext().provider;
    }
    
}
