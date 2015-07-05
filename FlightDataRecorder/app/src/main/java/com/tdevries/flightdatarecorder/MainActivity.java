/*
 * TODO
 * Turn off GPS and IMU sensors onStop
*/

package com.tdevries.flightdatarecorder;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.SystemClock;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.text.InputType;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Timer;
import java.util.TimerTask;


public class MainActivity extends ActionBarActivity implements SensorEventListener {

    public static final float EPSILON = 0.000000001f;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private float timestamp;
    private boolean initState = true;

    public static final int TIME_CONSTANT = 30; //30
    public static final float FILTER_COEFFICIENT = 0.98f; //0.98f
    private Timer fuseTimer;

    private SensorManager mSensorManager = null;

    // angular speeds from gyro
    private float[] gyro = new float[3];

    // rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];

    // orientation angles from gyro matrix
    private float[] gyroOrientation = new float[3];

    // magnetic field vector
    private float[] magnet = new float[3];

    // accelerometer vector
    private float[] accel = new float[3];

    // orientation angles from accel and magnet
    private float[] accMagOrientation = new float[3];

    // final orientation angles from sensor fusion
    private float[] fusedOrientation = new float[3];

    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];

    private LocationManager locationManager;
    private LocationListener locationListener;

    private TextView sensorText;
    private Button recordButton;

    final private String HEADER = "UTC Time (ms),Roll (deg),Pitch (deg),Azimuth (deg)," +
            "G-Force X, G-Force Y, G-Force Z," +
            "Latitude,Longitude,Ground Speed (m/s),Altitude (m),Bearing (deg)";
    private String orientationData, locationData;
    private String orientationNumeric, locationNumeric;
    private String dataForFile;

    private Timer dataRecordTimer;
    private TimerTask dataRecordTask;

    public int recordInterval = 1000;
    private boolean recording;
    private boolean fileReadyToTransfer;
    String filename = "dataFile1.txt";
    OutputStreamWriter outputStreamWriter;

    public String serverIP = "192.168.1.109";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        recording = false;
        fileReadyToTransfer = false;

        initViews();
        initGPS();
    }

    @Override
    protected void onStart(){
        super.onStart();
        //turn on everything that might be off
        //this method also automatically gets called when the app is created (because recording is set to false)
        if (!recording) {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);
            initIMU();
            initRecording();
            //Update roll, pitch, yaw.
            fuseTimer = new Timer();
            fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(), 1000, TIME_CONSTANT);
        }
    }

    @Override
    protected void onStop(){
        super.onStop();

        if (!recording){
            //turn off all the listeners so that battery is saved
            locationManager.removeUpdates(locationListener);
            mSensorManager.unregisterListener(this);
            fuseTimer.cancel();
            dataRecordTimer.cancel();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
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
        if (id == R.id.setServerIP){
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Set Server IP");
            builder.setMessage("If the data server IP address has changed enter the new address below:\n");

            // Set up the input
            final EditText input = new EditText(this);
            input.setText(serverIP);
            input.setGravity(Gravity.CENTER_HORIZONTAL);
            input.setInputType(InputType.TYPE_CLASS_TEXT);
            builder.setView(input);

            // Set up the buttons
            builder.setPositiveButton("Okay", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    serverIP = input.getText().toString();
                }
            });
            builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.cancel();
                }
            });

            builder.show();
            return true;
        }
        else if (id == R.id.setRecordInterval){
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Set Record Interval");
            builder.setMessage("Enter the interval between recordings (in ms):\n");

            // Set up the input
            final EditText input = new EditText(this);
            input.setText(String.valueOf(recordInterval));
            input.setGravity(Gravity.CENTER_HORIZONTAL);
            input.setInputType(InputType.TYPE_CLASS_TEXT);
            builder.setView(input);

            // Set up the buttons
            builder.setPositiveButton("Okay", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    recordInterval = Integer.parseInt(input.getText().toString());
                    //cancel old record timer and initialize a new one
                    dataRecordTimer.cancel();
                    initRecording();
                }
            });
            builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.cancel();
                }
            });

            builder.show();
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    public void initViews() {
        sensorText = (TextView) findViewById(R.id.sensorText);
        recordButton = (Button) findViewById(R.id.recordButton);
    }

    public void initIMU() {
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);
    }

    public void initGPS() {

        locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

        // Define a listener that responds to location updates
        locationListener = new LocationListener() {
            // Called when a new location is found by the network location provider.
            public void onLocationChanged(Location location) {
                double latitude = location.getLatitude();
                double longitude = location.getLongitude();
                //Ground speed in meter/second
                float groundSpeed = location.getSpeed();
                //Altitude is measured in meters above WGS 84 reference ellipsoid, needs more processing to be usable
                double altitude = location.getAltitude();
                float bearing = location.getBearing();

                locationData = "Latitude: " + latitude +
                        "\nLongitude: " + longitude +
                        "\nGround speed: " + groundSpeed +
                        "\nAltitude: " + altitude +
                        "\nBearing: " + bearing;

                locationNumeric = latitude +
                        "," + longitude +
                        "," + groundSpeed +
                        "," + altitude +
                        "," + bearing;

                sensorText.setText(orientationData + "\n" + locationData);
            }

            public void onStatusChanged(String provider, int status, Bundle extras) {
            }

            public void onProviderEnabled(String provider) {
            }

            public void onProviderDisabled(String provider) {
            }
        };

        // Register the listener with the Location Manager to receive location updates
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);
    }

    public void initRecording() {
        dataRecordTimer = new Timer();
        try {
            dataRecordTask = new TimerTask() {
                @Override
                public void run() {
                    //Log.i("Text", orientationData + "\n" + locationData);
                    if (recording == true) {
                        try {
                            String time = Long.toString(System.currentTimeMillis());
                            dataForFile = time + ',' + orientationNumeric + ',' + locationNumeric + '\n';
                            outputStreamWriter.write(dataForFile);
                        } catch (IOException e) {
                            Log.e("Exception", "File write failed: " + e.toString());
                        }
                    }
                    else if (fileReadyToTransfer == true){
                        ConnectivityManager connManager = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
                        NetworkInfo mWifi = connManager.getNetworkInfo(ConnectivityManager.TYPE_WIFI);

                        if (mWifi.isConnected()) {
                            Log.i("WiFi", "WiFi connected.");
                            SystemClock.sleep(1000);
                            readFromFile();
                        }
                    }
                }
            };
            //Initially wait 1 second, then run every RECORD_INTERVAL milliseconds after that
            dataRecordTimer.schedule(dataRecordTask, 1000, recordInterval);
        } catch (IllegalStateException e) {
            android.util.Log.i("Damn", "resume error");
        }
    }

    public void startStopRecording(View view) {
        recording = !recording;
        if (recording == true) {
            recordButton.setText("Stop Recording");

            try {
                Log.i("Start recording", "Recording started, data file created.");
                outputStreamWriter = new OutputStreamWriter(openFileOutput(filename, Context.MODE_APPEND));
            } catch (IOException e) {
                Log.e("Exception", "File write failed: " + e.toString());
            }
        } else if (recording == false) {
            recordButton.setText("Start Recording");

            try {
                Log.i("Stop recording", "Recording stopped, data file closed.");
                outputStreamWriter.close();
                fileReadyToTransfer = true;
            } catch (IOException e) {
                Log.e("Exception", "File write failed: " + e.toString());
            }
        }
    }

    private void readFromFile() {

        try {
            InputStream inputStream = openFileInput(filename);

            if (inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString = "";

                //http://www.roman10.net/android-tcp-client-and-server-communication-programmingillustrated-with-example/
                try {
                    Socket s = new Socket(serverIP, 5000);
                    BufferedReader in = new BufferedReader(new InputStreamReader(s.getInputStream()));
                    BufferedWriter out = new BufferedWriter(new OutputStreamWriter(s.getOutputStream()));

                    //send output msg
                    out.write(HEADER + "\n");
                    out.flush();

                    while ((receiveString = bufferedReader.readLine()) != null) {
                        out.write(receiveString + "\n");
                        out.flush();
                        Log.i("File output", receiveString);
                    }

                    //close connection
                    s.close();
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }

                inputStream.close();
            }
        } catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }
        //Delete the file after transmission so that we don't keep appending to it
        deleteFile(filename);
        fileReadyToTransfer = false;
    }

    public void calculateAccMagOrientation() {
        if (SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    private void getRotationVectorFromGyro(float[] gyroValues, float[] deltaRotationVector, float timeFactor) {
        float[] normValues = new float[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude =
                (float) Math.sqrt(gyroValues[0] * gyroValues[0] +
                        gyroValues[1] * gyroValues[1] +
                        gyroValues[2] * gyroValues[2]);

        // Normalize the rotation vector if it's big enough to get the axis
        if (omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }

        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float) Math.sin(o[1]);
        float cosX = (float) Math.cos(o[1]);
        float sinY = (float) Math.sin(o[2]);
        float cosY = (float) Math.cos(o[2]);
        float sinZ = (float) Math.sin(o[0]);
        float cosZ = (float) Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f;
        xM[1] = 0.0f;
        xM[2] = 0.0f;
        xM[3] = 0.0f;
        xM[4] = cosX;
        xM[5] = sinX;
        xM[6] = 0.0f;
        xM[7] = -sinX;
        xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY;
        yM[1] = 0.0f;
        yM[2] = sinY;
        yM[3] = 0.0f;
        yM[4] = 1.0f;
        yM[5] = 0.0f;
        yM[6] = -sinY;
        yM[7] = 0.0f;
        yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ;
        zM[1] = sinZ;
        zM[2] = 0.0f;
        zM[3] = -sinZ;
        zM[4] = cosZ;
        zM[5] = 0.0f;
        zM[6] = 0.0f;
        zM[7] = 0.0f;
        zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    public void gyroFunction(SensorEvent event) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;

        // initialisation of the gyroscope based rotation matrix
        if (initState) {
            float[] initMatrix = new float[9];
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }

        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if (timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
            System.arraycopy(event.values, 0, gyro, 0, 3);
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }

        // measurement done, save current time for next interval
        timestamp = event.timestamp;

        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        int roll = (int) Math.toDegrees((double) fusedOrientation[2]);
        int pitch = (int) Math.toDegrees((double) fusedOrientation[1]);
        int azimuth = (int) Math.toDegrees((double) fusedOrientation[0]);

        double gforceX = accel[0]/9.81;
        double gforceY = accel[1]/9.81;
        double gforceZ = accel[2]/9.81;

        orientationData = String.format( "Roll: %d \nPitch: %d \nAzimuth: %d \nG-Force X: %.1f \nG-Force Y: %.1f \nG-Force Z: %.1f",
                roll, pitch, azimuth, gforceX, gforceY, gforceZ );

        orientationNumeric = String.format( "%d,%d,%d,%.1f,%.1f,%.1f",
                roll, pitch, azimuth, gforceX, gforceY, gforceZ);

        sensorText.setText(orientationData + "\n" + locationData);

        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                // copy new accelerometer data into accel array
                // then calculate new orientation
                System.arraycopy(event.values, 0, accel, 0, 3);
                calculateAccMagOrientation();
                break;

            case Sensor.TYPE_GYROSCOPE:
                // process gyro data
                gyroFunction(event);
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                // copy new magnetometer data into magnet array
                System.arraycopy(event.values, 0, magnet, 0, 3);
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public class calculateFusedOrientationTask extends TimerTask {
        public void run() {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
            fusedOrientation[0] =
                    FILTER_COEFFICIENT * gyroOrientation[0]
                            + oneMinusCoeff * accMagOrientation[0];

            fusedOrientation[1] =
                    FILTER_COEFFICIENT * gyroOrientation[1]
                            + oneMinusCoeff * accMagOrientation[1];

            fusedOrientation[2] =
                    FILTER_COEFFICIENT * gyroOrientation[2]
                            + oneMinusCoeff * accMagOrientation[2];

            // overwrite gyro matrix and orientation with fused orientation
            // to comensate gyro drift
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);
        }
    }

}


