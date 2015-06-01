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
import android.widget.TextView;

public class MainActivity extends ActionBarActivity {

    private TextView text1;

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

                Log.i("Geo_Location", "Latitude: " + latitude + ", Longitude: " + longitude);
                Log.i("Geo Location", "Ground speed: " + groundSpeed + ", Altitude: " + altitude + ", Bearing: " + bearing);
                text1.setText("Latitude: " + latitude + "\nLongitude: " + longitude + "\nGround speed: " + groundSpeed + "\nAltitude: " + altitude + "\nBearing: " + bearing);
            }

            public void onStatusChanged(String provider, int status, Bundle extras) {}

            public void onProviderEnabled(String provider) {}

            public void onProviderDisabled(String provider) {}
        };

        // Register the listener with the Location Manager to receive location updates
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);
    }

    public void initializeViews() {
        text1 = (TextView)findViewById(R.id.text1);
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
}
