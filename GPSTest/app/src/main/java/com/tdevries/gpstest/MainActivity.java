package com.tdevries.gpstest;

import android.content.Context;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends ActionBarActivity {

    private TextView text1;
    private Button start_record, stop_record;

    private Timer timer = new Timer();
    private TimerTask timerTask;

    private String sensorData;
    private boolean recording;

    String filename = "dataFile.txt";
    OutputStreamWriter outputStreamWriter;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeViews();

        // Acquire a reference to the system Location Manager
        LocationManager locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

        // Define a listener that responds to location updates
        LocationListener locationListener = new LocationListener() {
            // Called when a new location is found by the network location provider.
            public void onLocationChanged(Location location) {
                double latitude = location.getLatitude();
                double longitude =  location.getLongitude();
                //Ground speed in meter/second
                float groundSpeed = location.getSpeed();
                //Altitude is measured in meters above WGS 84 reference ellipsoid, needs more processing to be usable
                double altitude = location.getAltitude();
                float bearing = location.getBearing();

                //Log.i("Geo_Location", "Latitude: " + latitude + ", Longitude: " + longitude);
                //Log.i("Geo Location", "Ground speed: " + groundSpeed + ", Altitude: " + altitude + ", Bearing: " + bearing);
                text1.setText("Latitude: " + latitude + "\nLongitude: " + longitude + "\nGround speed: " + groundSpeed + "\nAltitude: " + altitude + "\nBearing: " + bearing);
                sensorData = latitude + "," + longitude + "," + groundSpeed + "," + altitude + "," + bearing + "\n";
                Log.i("Geo Location", sensorData);
            }

            public void onStatusChanged(String provider, int status, Bundle extras) {}

            public void onProviderEnabled(String provider) {}

            public void onProviderDisabled(String provider) {}
        };

        // Register the listener with the Location Manager to receive location updates
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);
    }

    public void onResume(){
        super.onResume();
        try {
            timer = new Timer();
            timerTask = new TimerTask() {
                @Override
                public void run() {
                    //Run code here every 1 second
                    if (recording){
                        try {
                            Log.i("Logging", sensorData);
                            outputStreamWriter.write(sensorData);
                        }
                        catch (IOException e) {
                            Log.e("Exception", "File write failed: " + e.toString());
                        }
                    }
                }
            };
            timer.schedule(timerTask, 1000, 1000);
        } catch (IllegalStateException e){
            android.util.Log.i("Damn", "resume error");
        }
    }

    public void initializeViews() {
        text1 = (TextView)findViewById(R.id.text1);
        start_record = (Button)findViewById(R.id.start_record);
        stop_record = (Button)findViewById(R.id.stop_record);

        //Set recording off to start
        recording = false;
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    public void startRecording(View view){
        Log.i("Start recording", "Recording started, data file created.");
        recording = true;
        try {
            outputStreamWriter = new OutputStreamWriter(openFileOutput(filename, Context.MODE_APPEND));
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    public void stopRecording(View view){
        Log.i("Stop recording", "Recording stopped, data file closed.");
        try {
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
        recording = false;
        readFromFile();
    }

    private void readFromFile() {

        String ret = "";

        try {
            InputStream inputStream = openFileInput(filename);

            if ( inputStream != null ) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString = "";
                StringBuilder stringBuilder = new StringBuilder();

                while ( (receiveString = bufferedReader.readLine()) != null ) {
                    stringBuilder.append(receiveString);
                    Log.i("File output", receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        }
        catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }
        //Delete the file after transmission so that we don't keep appending to it
        deleteFile(filename);
    }
}
