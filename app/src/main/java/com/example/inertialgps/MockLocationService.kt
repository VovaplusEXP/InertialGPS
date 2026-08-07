package com.example.inertialgps

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import android.os.SystemClock
import android.util.Log
import androidx.core.app.NotificationCompat
import kotlin.math.cos
import kotlin.math.PI

class MockLocationService : Service(), SensorEventListener, LocationListener {

    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager

    private var linearAccelerationSensor: Sensor? = null
    private var rotationVectorSensor: Sensor? = null

    private var initialLat = 0.0
    private var initialLon = 0.0
    private var gotInitialLocation = false

    private var lastTime: Long = 0
    private var velocity = FloatArray(3) // x, y, z in Earth frame (East, North, Up)
    private var positionOffset = FloatArray(3) // x, y, z in Earth frame (East, North, Up)

    private val rotationMatrix = FloatArray(9)
    private var hasRotation = false
    
    private val CHANNEL_ID = "MockLocationServiceChannel"

    override fun onCreate() {
        super.onCreate()
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager

        linearAccelerationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("Inertial GPS")
            .setContentText("Running mock location...")
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .build()
        startForeground(1, notification)

        // Request initial location
        try {
            val lastLoc = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER)
                ?: locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER)
            if (lastLoc != null) {
                setInitialLocation(lastLoc)
            } else {
                locationManager.requestSingleUpdate(LocationManager.GPS_PROVIDER, this, null)
            }
        } catch (e: SecurityException) {
            e.printStackTrace()
        }

        return START_STICKY
    }

    private fun setInitialLocation(location: Location) {
        if (gotInitialLocation) return
        initialLat = location.latitude
        initialLon = location.longitude
        gotInitialLocation = true

        setupMockProvider()
        startSensors()
    }

    private fun setupMockProvider() {
        try {
            locationManager.addTestProvider(
                LocationManager.GPS_PROVIDER, false, false, false, false,
                true, true, true, 0, 1
            )
            locationManager.setTestProviderEnabled(LocationManager.GPS_PROVIDER, true)
        } catch (e: Exception) {
            Log.e("MockLocationService", "Error setting up mock provider. Is it selected in Developer Options?", e)
        }
    }

    private fun startSensors() {
        lastTime = System.currentTimeMillis()
        sensorManager.registerListener(this, linearAccelerationSensor, SensorManager.SENSOR_DELAY_GAME)
        sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_GAME)
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!gotInitialLocation) return

        if (event.sensor.type == Sensor.TYPE_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
            hasRotation = true
        } else if (event.sensor.type == Sensor.TYPE_LINEAR_ACCELERATION && hasRotation) {
            val currentTime = System.currentTimeMillis()
            val dt = (currentTime - lastTime) / 1000f // seconds
            lastTime = currentTime

            // Transform local phone linear acceleration to Earth frame (East, North, Up)
            val ax = event.values[0]
            val ay = event.values[1]
            val az = event.values[2]

            // Matrix-vector multiplication
            val earthAx = rotationMatrix[0] * ax + rotationMatrix[1] * ay + rotationMatrix[2] * az // East
            val earthAy = rotationMatrix[3] * ax + rotationMatrix[4] * ay + rotationMatrix[5] * az // North
            val earthAz = rotationMatrix[6] * ax + rotationMatrix[7] * ay + rotationMatrix[8] * az // Up

            // Integrate acceleration to velocity
            velocity[0] += earthAx * dt
            velocity[1] += earthAy * dt
            velocity[2] += earthAz * dt

            // Apply artificial dampening to prevent infinite drift (since pure INS drifts terribly)
            // This simulates friction / walking characteristics. It resets velocity to 0 very quickly if no accel.
            val dampening = 0.95f
            velocity[0] *= dampening
            velocity[1] *= dampening
            velocity[2] *= dampening

            // Integrate velocity to position
            positionOffset[0] += velocity[0] * dt // East displacement (meters)
            positionOffset[1] += velocity[1] * dt // North displacement (meters)

            updateMockLocation()
        }
    }

    private fun updateMockLocation() {
        // Convert offset in meters to lat/lon
        // 1 degree latitude is approx 111,111 meters
        val latOffset = positionOffset[1] / 111111.0
        val newLat = initialLat + latOffset

        // 1 degree longitude is approx 111,111 * cos(latitude) meters
        val lonOffset = positionOffset[0] / (111111.0 * cos(initialLat * PI / 180.0))
        val newLon = initialLon + lonOffset

        val mockLocation = Location(LocationManager.GPS_PROVIDER).apply {
            latitude = newLat
            longitude = newLon
            altitude = 0.0
            time = System.currentTimeMillis()
            elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
            accuracy = 5.0f
        }

        try {
            locationManager.setTestProviderLocation(LocationManager.GPS_PROVIDER, mockLocation)
            
            // Send update to UI
            val intent = Intent("LOCATION_UPDATE").apply {
                putExtra("lat", newLat)
                putExtra("lon", newLon)
            }
            sendBroadcast(intent)
        } catch (e: Exception) {
            Log.e("MockLocationService", "Error setting mock location", e)
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    override fun onLocationChanged(location: Location) {
        setInitialLocation(location)
    }
    @Deprecated("Deprecated in Java")
    override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {}
    override fun onProviderEnabled(provider: String) {}
    override fun onProviderDisabled(provider: String) {}

    override fun onDestroy() {
        super.onDestroy()
        sensorManager.unregisterListener(this)
        try {
            locationManager.removeTestProvider(LocationManager.GPS_PROVIDER)
        } catch (e: Exception) {}
    }

    override fun onBind(intent: Intent?): IBinder? = null

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val serviceChannel = NotificationChannel(
                CHANNEL_ID,
                "Mock Location Service Channel",
                NotificationManager.IMPORTANCE_LOW
            )
            val manager = getSystemService(NotificationManager::class.java)
            manager?.createNotificationChannel(serviceChannel)
        }
    }
}
