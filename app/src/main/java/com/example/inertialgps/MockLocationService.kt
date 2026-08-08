package com.example.inertialgps

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import androidx.core.app.NotificationCompat
import kotlin.math.cos
import kotlin.math.PI

class MockLocationService : Service(), SensorEventListener, LocationListener {

    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager
    private lateinit var prefs: SharedPreferences

    private var linearAccelerationSensor: Sensor? = null
    private var rotationVectorSensor: Sensor? = null

    private var initialLat = 0.0
    private var initialLon = 0.0
    private var gotInitialLocation = false

    private var lastTime: Long = 0
    private var velocity = FloatArray(3) // x, y, z in Earth frame
    private var positionOffset = FloatArray(3) // x, y, z in Earth frame

    private val rotationMatrix = FloatArray(9)
    private var hasRotation = false
    private var isInertialMode = false
    
    // Calibration
    private var isCalibrating = false
    private var biasX = 0f
    private var biasY = 0f
    private var biasZ = 0f
    private var calibSumX = 0f
    private var calibSumY = 0f
    private var calibSumZ = 0f
    private var calibCount = 0
    
    private val CHANNEL_ID = "MockLocationServiceChannel"

    override fun onCreate() {
        super.onCreate()
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        prefs = getSharedPreferences("InertialGPS", Context.MODE_PRIVATE)
        
        biasX = prefs.getFloat("biasX", 0f)
        biasY = prefs.getFloat("biasY", 0f)
        biasZ = prefs.getFloat("biasZ", 0f)

        linearAccelerationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        if (rotationVectorSensor == null) {
            rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
        }

        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("Inertial GPS")
            .setContentText("Service running...")
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setOngoing(true)
            .build()
        startForeground(1, notification)

        when (intent?.action) {
            "START_SERVICE" -> {
                isInertialMode = false
                // Register sensors immediately if we want to allow calibration without enabling inertial mode
                sensorManager.registerListener(this, linearAccelerationSensor, SensorManager.SENSOR_DELAY_GAME)
                sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_GAME)
            }
            "ENABLE_INERTIAL" -> {
                enableInertialMode()
            }
            "DISABLE_INERTIAL" -> {
                disableInertialMode()
            }
            "START_CALIBRATION" -> {
                startCalibration()
            }
        }

        return START_STICKY
    }
    
    private fun startCalibration() {
        if (isCalibrating) return
        isCalibrating = true
        calibSumX = 0f
        calibSumY = 0f
        calibSumZ = 0f
        calibCount = 0
        
        Handler(Looper.getMainLooper()).postDelayed({
            isCalibrating = false
            if (calibCount > 0) {
                biasX = calibSumX / calibCount
                biasY = calibSumY / calibCount
                biasZ = calibSumZ / calibCount
                
                prefs.edit()
                    .putFloat("biasX", biasX)
                    .putFloat("biasY", biasY)
                    .putFloat("biasZ", biasZ)
                    .apply()
                    
                val doneIntent = Intent("com.example.inertialgps.CALIBRATION_DONE").apply {
                    putExtra("biasX", biasX)
                    putExtra("biasY", biasY)
                    putExtra("biasZ", biasZ)
                }
                sendBroadcast(doneIntent)
            }
        }, 5000)
    }

    private fun enableInertialMode() {
        if (isInertialMode) return
        isInertialMode = true
        gotInitialLocation = false

        try {
            val lastLoc = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER)
                ?: locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER)
            
            if (lastLoc != null) {
                setInitialLocation(lastLoc)
            } else {
                sendBroadcast(Intent("com.example.inertialgps.GPS_WAITING"))
                locationManager.requestSingleUpdate(LocationManager.GPS_PROVIDER, this, null)
            }
        } catch (e: SecurityException) {
            e.printStackTrace()
        }
    }

    private fun disableInertialMode() {
        if (!isInertialMode) return
        isInertialMode = false
        
        try {
            locationManager.removeTestProvider(LocationManager.GPS_PROVIDER)
        } catch (e: Exception) {
            Log.e("MockLocationService", "Error removing mock provider", e)
        }
    }

    private fun setInitialLocation(location: Location) {
        if (gotInitialLocation || !isInertialMode) return
        
        initialLat = location.latitude
        initialLon = location.longitude
        
        velocity.fill(0f)
        positionOffset.fill(0f)
        
        gotInitialLocation = true
        lastTime = System.currentTimeMillis()
        setupMockProvider()
    }

    private fun setupMockProvider() {
        try {
            locationManager.addTestProvider(
                LocationManager.GPS_PROVIDER, false, false, false, false,
                true, true, true, 0, 1
            )
            locationManager.setTestProviderEnabled(LocationManager.GPS_PROVIDER, true)
        } catch (e: Exception) {
            Log.e("MockLocationService", "Error setting up mock provider", e)
            sendBroadcast(Intent("com.example.inertialgps.MOCK_DENIED").apply { setPackage(packageName) })
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type == Sensor.TYPE_ROTATION_VECTOR || event.sensor.type == Sensor.TYPE_GAME_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
            hasRotation = true
        } else if (event.sensor.type == Sensor.TYPE_LINEAR_ACCELERATION) {
            val axRaw = event.values[0]
            val ayRaw = event.values[1]
            val azRaw = event.values[2]
            
            if (isCalibrating) {
                calibSumX += axRaw
                calibSumY += ayRaw
                calibSumZ += azRaw
                calibCount++
            }
            
            if (!gotInitialLocation || !isInertialMode || !hasRotation) return

            val ax = axRaw - biasX
            val ay = ayRaw - biasY
            val az = azRaw - biasZ

            val currentTime = System.currentTimeMillis()
            val dt = (currentTime - lastTime) / 1000f
            lastTime = currentTime
            
            if (dt > 0.5f) return

            val earthAx = rotationMatrix[0] * ax + rotationMatrix[1] * ay + rotationMatrix[2] * az
            val earthAy = rotationMatrix[3] * ax + rotationMatrix[4] * ay + rotationMatrix[5] * az
            val earthAz = rotationMatrix[6] * ax + rotationMatrix[7] * ay + rotationMatrix[8] * az

            velocity[0] += earthAx * dt
            velocity[1] += earthAy * dt
            velocity[2] += earthAz * dt

            val dampening = 0.98f
            velocity[0] *= dampening
            velocity[1] *= dampening
            velocity[2] *= dampening

            positionOffset[0] += velocity[0] * dt
            positionOffset[1] += velocity[1] * dt

            updateMockLocation()
        }
    }

    private fun updateMockLocation() {
        val latOffset = positionOffset[1] / 111111.0
        val newLat = initialLat + latOffset
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

        // Always broadcast to UI regardless of Mock Provider status
        val intent = Intent("com.example.inertialgps.LOCATION_UPDATE").apply {
            setPackage(packageName)
            putExtra("lat", newLat)
            putExtra("lon", newLon)
        }
        sendBroadcast(intent)

        try {
            locationManager.setTestProviderLocation(LocationManager.GPS_PROVIDER, mockLocation)
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
        disableInertialMode()
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
