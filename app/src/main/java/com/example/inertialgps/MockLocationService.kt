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
import androidx.core.content.ContextCompat
import kotlin.math.cos
import kotlin.math.PI

class MockLocationService : Service(), SensorEventListener, LocationListener {

    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager
    private lateinit var prefs: SharedPreferences

    private var linearAccelerationSensor: Sensor? = null
    private var rawAccelSensor: Sensor? = null
    private var gyroSensor: Sensor? = null
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
    
    private val stepDetector = StepDetector()
    private val CHANNEL_ID = "MockLocationServiceChannel"

    override fun onCreate() {
        super.onCreate()
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        prefs = getSharedPreferences("InertialGPS", Context.MODE_PRIVATE)
        prefs.edit().putBoolean("isServiceRunning", true).apply()
        
        biasX = prefs.getFloat("biasX", 0f)
        biasY = prefs.getFloat("biasY", 0f)
        biasZ = prefs.getFloat("biasZ", 0f)

        linearAccelerationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rawAccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) ?: sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) ?: sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        
        // Use Game Rotation Vector (immune to indoor magnetic tilt interference)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
        if (rotationVectorSensor == null) {
            rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
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
                sensorManager.registerListener(this, linearAccelerationSensor, SensorManager.SENSOR_DELAY_FASTEST)
                sensorManager.registerListener(this, rawAccelSensor, SensorManager.SENSOR_DELAY_FASTEST)
                sensorManager.registerListener(this, gyroSensor, SensorManager.SENSOR_DELAY_FASTEST)
                sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_FASTEST)
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
        // Obsolete: Dynamic ESKF ZUPT handles this better.
        val doneIntent = Intent("com.example.inertialgps.CALIBRATION_DONE").apply {
            setPackage(packageName)
        }
        sendBroadcast(doneIntent)
    }

    private fun enableInertialMode() {
        if (isInertialMode) return
        isInertialMode = true
        prefs.edit().putBoolean("isInertialEnabled", true).apply()
        gotInitialLocation = false

        try {
            sendBroadcast(Intent("com.example.inertialgps.GPS_WAITING"))
            // Force a fresh, high-accuracy GPS update
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                locationManager.getCurrentLocation(
                    LocationManager.GPS_PROVIDER,
                    null,
                    ContextCompat.getMainExecutor(this),
                    { location ->
                        if (location != null) setInitialLocation(location)
                        else locationManager.requestSingleUpdate(LocationManager.GPS_PROVIDER, this, null)
                    }
                )
            } else {
                locationManager.requestSingleUpdate(LocationManager.GPS_PROVIDER, this, null)
            }
        } catch (e: SecurityException) {
            e.printStackTrace()
        }
    }

    private fun disableInertialMode() {
        if (!isInertialMode) return
        isInertialMode = false
        prefs.edit().putBoolean("isInertialEnabled", false).apply()
        
        // Disable sensors when not walking to save battery
        sensorManager.unregisterListener(this, rawAccelSensor)
        sensorManager.unregisterListener(this, linearAccelerationSensor)
        
        // Force cleanup of all possible providers (in case they were orphaned by previous app builds)
        val allPossibleProviders = arrayOf(LocationManager.GPS_PROVIDER, LocationManager.NETWORK_PROVIDER, "fused")
        for (provider in allPossibleProviders) {
            try {
                locationManager.removeTestProvider(provider)
            } catch (e: Exception) {
                Log.e("MockLocationService", "Cleanup: Provider $provider not removed", e)
            }
        }
    }

    private fun setInitialLocation(location: Location) {
        if (gotInitialLocation || !isInertialMode) return
        
        initialLat = location.latitude
        initialLon = location.longitude
        
        velocity.fill(0f)
        positionOffset.fill(0f)
        
        // Hard Reset ESKF
        eskf.position.fill(0.0)
        eskf.velocity.fill(0.0)
        isEskfInitialized = false
        pdrX = 0.0
        pdrY = 0.0
        
        gotInitialLocation = true
        lastTime = System.currentTimeMillis()
        setupMockProvider()
    }

    private val MOCK_PROVIDERS = arrayOf(
        LocationManager.GPS_PROVIDER,
        LocationManager.NETWORK_PROVIDER,
        "fused"
    )

    private fun setupMockProvider() {
        for (provider in MOCK_PROVIDERS) {
            try {
                try {
                    locationManager.removeTestProvider(provider)
                } catch (e: Exception) {}

                locationManager.addTestProvider(
                    provider, false, false, false, false,
                    true, true, true, 0, 1
                )
                locationManager.setTestProviderEnabled(provider, true)
            } catch (e: SecurityException) {
                Log.e("MockLocationService", "SecurityException: Mock provider permission denied", e)
                sendBroadcast(Intent("com.example.inertialgps.MOCK_DENIED").apply { setPackage(packageName) })
                return
            } catch (e: IllegalArgumentException) {
                Log.e("MockLocationService", "IllegalArgumentException setting up mock provider $provider", e)
            } catch (e: Exception) {
                Log.e("MockLocationService", "Error setting up mock provider $provider", e)
            }
        }
    }

    private var pdrX = 0.0
    private var pdrY = 0.0

    private var currentGyro = FloatArray(3)
    private var stepsTaken = 0
    private var timeSinceLastStep = 0L
    private var lastMockUpdateTime = 0L
    
    private val eskf = ESKF()
    private var isEskfInitialized = false

    override fun onSensorChanged(event: SensorEvent) {
        val type = event.sensor.type
        val currentTime = System.currentTimeMillis()
        
        if (type == Sensor.TYPE_ROTATION_VECTOR || type == Sensor.TYPE_GAME_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
            hasRotation = true
            
            if (!isEskfInitialized) {
                val q = FloatArray(4)
                SensorManager.getQuaternionFromVector(q, event.values)
                eskf.quaternion.set(0, 0, q[0].toDouble())
                eskf.quaternion.set(1, 0, q[1].toDouble())
                eskf.quaternion.set(2, 0, q[2].toDouble())
                eskf.quaternion.set(3, 0, q[3].toDouble())
                isEskfInitialized = true
            }
            
        } else if (type == Sensor.TYPE_GYROSCOPE_UNCALIBRATED || type == Sensor.TYPE_GYROSCOPE) {
            currentGyro[0] = event.values[0]
            currentGyro[1] = event.values[1]
            currentGyro[2] = event.values[2]
        } else if (type == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED || type == Sensor.TYPE_ACCELEROMETER) {
            
            if (!gotInitialLocation || !isInertialMode || !isEskfInitialized) return
            
            if (lastTime == 0L) {
                lastTime = event.timestamp
                return
            }
            
            val dt = (event.timestamp - lastTime) / 1_000_000_000.0
            lastTime = event.timestamp
            
            if (dt > 0.5 || dt <= 0.0) return
            
            try {
                eskf.predict(event.values, currentGyro, dt)
                
                val isStep = stepDetector.process(eskf.linearAccelWorld, currentGyro[0], currentGyro[1], currentGyro[2], currentTime)
                
                if (isStep) {
                    stepsTaken++
                    timeSinceLastStep = currentTime
                    
                    // Path B: True Physical Heading via ESKF Velocity
                    val vx = eskf.velocity.get(0, 0)
                    val vy = eskf.velocity.get(1, 0)
                    
                    // Fallback to compass if velocity is too small to determine direction
                    val heading = if (kotlin.math.abs(vx) > 0.1 || kotlin.math.abs(vy) > 0.1) {
                        kotlin.math.atan2(vy, vx)
                    } else {
                        kotlin.math.atan2(rotationMatrix[4].toDouble(), rotationMatrix[1].toDouble())
                    }

                    val stepLength = stepDetector.stepLength
                    val dx = stepLength * kotlin.math.cos(heading)
                    val dy = stepLength * kotlin.math.sin(heading)
                    
                    pdrX += dx
                    pdrY += dy
                    
                    val measuredPos = doubleArrayOf(
                        pdrX,
                        pdrY,
                        eskf.position.get(2, 0) // Assume z doesn't change much for PDR
                    )
                    
                    eskf.updatePosition(measuredPos, 0.01)
                    
                    val logMsg = String.format("Step %d: L=%.2fm, H=%.1f°, dx=%.2f, dy=%.2f", stepsTaken, stepLength, heading * 180.0 / Math.PI, dx, dy)
                    sendBroadcast(Intent("com.example.inertialgps.PDR_LOG").apply {
                        setPackage(packageName)
                        putExtra("log", logMsg)
                    })
                    
                } else {
                    if (currentTime - timeSinceLastStep > 2000) {
                        val ax = event.values[0]
                        val ay = event.values[1]
                        val az = event.values[2]
                        val accelVariance = kotlin.math.sqrt(ax*ax + ay*ay + az*az)
                        
                        if (kotlin.math.abs(accelVariance - 9.81f) < 0.3f) {
                            eskf.updateZUPT(0.5)
                        }
                    }
                }
                
                // Throttle UI and Mock Location updates to prevent Broadcast flooding (1 Hz or on Step)
                if (currentTime - lastMockUpdateTime > 1000 || isStep) {
                    
                    if (currentTime - lastMockUpdateTime > 1000) {
                        val v = eskf.velocity
                        val p = eskf.position
                        val b = eskf.accelBias
                        val zupt = if (currentTime - timeSinceLastStep > 2000 && kotlin.math.abs(kotlin.math.sqrt(event.values[0]*event.values[0] + event.values[1]*event.values[1] + event.values[2]*event.values[2]) - 9.81f) < 0.3f) "ZUPT Active" else "Moving"
                        val sysLog = String.format("Pos: %.2f, %.2f, %.2f\nVel: %.2f, %.2f, %.2f\nBias: %.3f, %.3f, %.3f\nState: %s", 
                            p.get(0,0), p.get(1,0), p.get(2,0),
                            v.get(0,0), v.get(1,0), v.get(2,0),
                            b.get(0,0), b.get(1,0), b.get(2,0),
                            zupt)
                        sendBroadcast(Intent("com.example.inertialgps.SYS_LOG").apply {
                            setPackage(packageName)
                            putExtra("log", sysLog)
                        })
                    }
                    
                    lastMockUpdateTime = currentTime
                    
                    positionOffset[0] = eskf.position.get(0, 0).toFloat()
                    positionOffset[1] = eskf.position.get(1, 0).toFloat()
                    
                    updateMockLocation()
                }
                
            } catch (e: Exception) {
                // Catch matrix math errors and broadcast to UI
                val errorIntent = Intent("com.example.inertialgps.MOCK_DENIED")
                errorIntent.setPackage(packageName)
                sendBroadcast(errorIntent)
            }
        }
    }

    private fun updateMockLocation() {
        val latOffset = positionOffset[1] / 111111.0
        val newLat = initialLat + latOffset
        val lonOffset = positionOffset[0] / (111111.0 * cos(initialLat * PI / 180.0))
        val newLon = initialLon + lonOffset

        val currentSpeed = kotlin.math.sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1])
        val currentBearing = ((kotlin.math.atan2(velocity[0], velocity[1]) * 180 / PI + 360) % 360).toFloat()

        val baseLocation = Location(LocationManager.GPS_PROVIDER).apply {
            latitude = newLat
            longitude = newLon
            altitude = 0.0
            speed = currentSpeed
            bearing = currentBearing
            time = System.currentTimeMillis()
            elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
            accuracy = 5.0f
        }

        val intent = Intent("com.example.inertialgps.LOCATION_UPDATE").apply {
            setPackage(packageName)
            putExtra("lat", newLat)
            putExtra("lon", newLon)
        }
        sendBroadcast(intent)

        for (provider in MOCK_PROVIDERS) {
            try {
                val mockLocation = Location(baseLocation)
                mockLocation.provider = provider
                locationManager.setTestProviderLocation(provider, mockLocation)
            } catch (e: Exception) {
                // Ignore
            }
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
        prefs.edit().putBoolean("isServiceRunning", false).putBoolean("isInertialEnabled", false).apply()
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
