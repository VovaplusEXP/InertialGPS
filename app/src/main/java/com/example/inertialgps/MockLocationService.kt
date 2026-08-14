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
import android.os.HandlerThread
import android.os.IBinder
import android.os.Process
import android.os.SystemClock
import android.util.Log
import androidx.core.app.NotificationCompat
import androidx.core.content.ContextCompat
import com.example.inertialgps.gnss.AndroidGnssAdapter
import kotlin.math.cos
import kotlin.math.PI

class MockLocationService : Service(), SensorEventListener, LocationListener {

    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager
    private lateinit var prefs: SharedPreferences

    private var rawAccelSensor: Sensor? = null
    private var gyroSensor: Sensor? = null
    private var rotationVectorSensor: Sensor? = null
    private var pressureSensor: Sensor? = null
    private var isBarometerAvailable = false
    private var initialPressure = 0.0f
    private var currentPressure = 0.0f
    private var filteredBaroAlt = 0.0

    private var sensorThread: HandlerThread? = null
    private var sensorHandler: Handler? = null

    private var initialLat = 0.0
    private var initialLon = 0.0
    private var initialAlt = 0.0
    @Volatile private var gotInitialLocation = false

    private var lastTime: Long = 0
    private var velocity = FloatArray(3) // x, y, z in Earth frame
    private var positionOffset = FloatArray(3) // x, y, z in Earth frame
    private val currentLla = DoubleArray(3) // lat, lon, alt result buffer

    private val rotationMatrix = FloatArray(9)
    private var hasRotation = false
    @Volatile private var isInertialMode = false
    
    private val stepDetector = StepDetector()
    private val CHANNEL_ID = "MockLocationServiceChannel"
    private lateinit var gnssAdapter: AndroidGnssAdapter

    // GNSS Doppler Fusion tracking
    private var gnssFusionCount = 0
    private var lastGnssFusionTime = 0L

    override fun onCreate() {
        super.onCreate()
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        prefs = getSharedPreferences("InertialGPS", Context.MODE_PRIVATE)
        prefs.edit().putBoolean("isServiceRunning", true).apply()
        
        gnssAdapter = AndroidGnssAdapter(this)
        gnssAdapter.setOnVelocityListener { gnssVel ->
            sensorHandler?.post {
                if (!isInertialMode || !isEskfInitialized || !gotInitialLocation) return@post
                
                // Loosely-Coupled SINS/GNSS Integration:
                // Gate incoming velocity: reject high covariance / noise (> 2.5 m/s) and NaNs
                if (gnssVel.covariance < 2.5 && !gnssVel.vx.isNaN() && !gnssVel.vy.isNaN() && !gnssVel.vz.isNaN()) {
                    // Measurement noise covariance R: square of 1-sigma uncertainty, minimum 0.04 (m/s)^2
                    val rCov = kotlin.math.max(gnssVel.covariance * gnssVel.covariance, 0.04)
                    val velArray = doubleArrayOf(gnssVel.vx, gnssVel.vy, gnssVel.vz)
                    
                    eskf.updateVelocity(velArray, rCov)
                    gnssFusionCount++
                    lastGnssFusionTime = System.currentTimeMillis()
                }
            }
        }

        val thread = HandlerThread("InertialSensorThread", Process.THREAD_PRIORITY_MORE_FAVORABLE)
        thread.start()
        sensorThread = thread
        sensorHandler = Handler(thread.looper)
        
        rawAccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) 
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) 
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        
        // Use Game Rotation Vector (immune to indoor magnetic tilt interference)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        // Barometer (Optional on non-flagships)
        pressureSensor = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE)
        isBarometerAvailable = (pressureSensor != null)

        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (intent == null) {
            val wasRunning = prefs.getBoolean("isServiceRunning", false)
            val wasInertial = prefs.getBoolean("isInertialEnabled", false)
            if (wasRunning) {
                registerSensors()
                if (wasInertial) {
                    enableInertialMode()
                }
                updateNotification()
                return START_STICKY
            }
        }

        if (intent?.action == "STOP_SERVICE") {
            stopSelf()
            return START_NOT_STICKY
        }

        updateNotification()

        when (intent?.action) {
            "START_SERVICE" -> {
                isInertialMode = false
                registerSensors()
                updateNotification()
            }
            "ENABLE_INERTIAL" -> {
                enableInertialMode()
                updateNotification()
            }
            "DISABLE_INERTIAL" -> {
                disableInertialMode()
                updateNotification()
            }
        }
        return START_STICKY
    }

    private fun updateNotification() {
        val openAppIntent = Intent(this, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_SINGLE_TOP
        }
        val openAppPendingIntent = android.app.PendingIntent.getActivity(
            this,
            0,
            openAppIntent,
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                android.app.PendingIntent.FLAG_IMMUTABLE or android.app.PendingIntent.FLAG_UPDATE_CURRENT
            } else {
                android.app.PendingIntent.FLAG_UPDATE_CURRENT
            }
        )

        val stopIntent = Intent(this, MockLocationService::class.java).apply {
            action = "STOP_SERVICE"
        }
        val stopPendingIntent = android.app.PendingIntent.getService(
            this,
            1,
            stopIntent,
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                android.app.PendingIntent.FLAG_IMMUTABLE or android.app.PendingIntent.FLAG_UPDATE_CURRENT
            } else {
                android.app.PendingIntent.FLAG_UPDATE_CURRENT
            }
        )

        val contentText = if (isInertialMode) "Inertial Fusion Mode Active" else "Mock GPS Provider Active"

        val notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("Inertial GPS")
            .setContentText(contentText)
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setContentIntent(openAppPendingIntent)
            .addAction(android.R.drawable.ic_menu_close_clear_cancel, "Stop Service", stopPendingIntent)
            .setOngoing(true)
            .build()

        startForeground(1, notification)
    }

    private fun registerSensors() {
        val handler = sensorHandler
        sensorManager.registerListener(this, rawAccelSensor, SensorManager.SENSOR_DELAY_FASTEST, handler)
        sensorManager.registerListener(this, gyroSensor, SensorManager.SENSOR_DELAY_FASTEST, handler)
        sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_FASTEST, handler)
        if (pressureSensor != null) {
            sensorManager.registerListener(this, pressureSensor, SensorManager.SENSOR_DELAY_NORMAL, handler)
        }
    }

    private fun unregisterSensors() {
        sensorManager.unregisterListener(this)
    }

    private fun enableInertialMode() {
        if (isInertialMode) return
        isInertialMode = true
        prefs.edit().putBoolean("isInertialEnabled", true).apply()
        gotInitialLocation = false
        gnssFusionCount = 0
        lastGnssFusionTime = 0L
        gnssAdapter.start()
        registerSensors()

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
        
        gnssAdapter.stop()
        unregisterSensors()
        
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
        sensorHandler?.post {
            setInitialLocationInternal(location)
        } ?: setInitialLocationInternal(location)
    }

    private fun setInitialLocationInternal(location: Location) {
        if (gotInitialLocation || !isInertialMode) return
        
        initialLat = location.latitude
        initialLon = location.longitude
        initialAlt = location.altitude
        
        initialPressure = if (currentPressure > 0f) currentPressure else 0f
        filteredBaroAlt = 0.0

        velocity.fill(0f)
        positionOffset.fill(0f)
        
        // Feed reference position to GNSS adapter solver
        gnssAdapter.setReferencePosition(location.latitude, location.longitude, location.altitude)

        // Hard Reset ESKF
        eskf.reset()
        isEskfInitialized = false
        
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

    private var currentGyro = FloatArray(3)
    
    private var stepsTaken = 0
    private var timeSinceLastStep = 0L
    private var lastMockUpdateTime = 0L
    
    // ZUPT Window
    private val accelWindow = FloatArray(15)
    private var windowIndex = 0
    private var isWindowFull = false
    private var isZuptActive = false
    
    // NHC Gyro-Aided Freeze variables
    private var savedBodyAxisX = 0.0
    private var savedBodyAxisY = 1.0
    private var hasSavedBodyAxis = false
    private var currentNhcHeading = 0.0
    
    // Step Gating
    private var consecutiveSteps = 0
    
    private val eskf = ESKF()
    private var isEskfInitialized = false

    override fun onSensorChanged(event: SensorEvent) {
        val type = event.sensor.type
        val currentTime = System.currentTimeMillis()
        
        if (type == Sensor.TYPE_ROTATION_VECTOR || type == Sensor.TYPE_GAME_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
            hasRotation = true
            
            if (!isEskfInitialized) {
                isEskfInitialized = true
            }
            
        } else if (type == Sensor.TYPE_GYROSCOPE_UNCALIBRATED || type == Sensor.TYPE_GYROSCOPE) {
            currentGyro[0] = event.values[0]
            currentGyro[1] = event.values[1]
            currentGyro[2] = event.values[2]
        } else if (type == Sensor.TYPE_PRESSURE) {
            currentPressure = event.values[0]
            if (gotInitialLocation && isInertialMode && isEskfInitialized && currentPressure > 0f) {
                if (initialPressure <= 0f) {
                    initialPressure = currentPressure
                }
                if (initialPressure > 0f) {
                    // Hypsometric relative altitude displacement (in meters)
                    val rawDeltaH = 44330.0 * (1.0 - Math.pow((currentPressure / initialPressure).toDouble(), 0.190295))
                    // Low-pass filter to reject indoor door-slam / AC pressure waves
                    filteredBaroAlt = filteredBaroAlt * 0.9 + rawDeltaH * 0.1
                    // Soft update into ESKF Z-position state
                    eskf.updateAltitude(filteredBaroAlt, 1.0)
                }
            }
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
                eskf.predict(event.values, rotationMatrix, dt)
                
                val vx = eskf.velocity.x
                val vy = eskf.velocity.y
                
                val isStep = stepDetector.process(eskf.linearAccelWorld, currentGyro[0], currentGyro[1], currentGyro[2], vx.toFloat(), vy.toFloat(), currentTime)
                
                // ==========================================
                // HYBRID SINS + PDR STATE MACHINE
                // ==========================================
                
                // STATE 1: ZUPT (Stationary)
                val ax = event.values[0]
                val ay = event.values[1]                                          
                val az = event.values[2]
                val magnitude = kotlin.math.sqrt(ax*ax + ay*ay + az*az)
                
                accelWindow[windowIndex] = magnitude
                windowIndex = (windowIndex + 1) % accelWindow.size
                if (windowIndex == 0) isWindowFull = true
                
                isZuptActive = false
                if (isWindowFull && (currentTime - timeSinceLastStep > 1200)) {
                    var minMag = accelWindow[0]
                    var maxMag = accelWindow[0]
                    
                    for (i in 1 until accelWindow.size) {
                        if (accelWindow[i] < minMag) minMag = accelWindow[i]
                        if (accelWindow[i] > maxMag) maxMag = accelWindow[i]
                    }
                    val signalSpread = maxMag - minMag
                    if (signalSpread < 0.18f) { 
                        eskf.updateZUPT(0.01)
                        isZuptActive = true
                        consecutiveSteps = 0
                    }
                }

                // STATE 2: PDR (Walking)
                if (isStep) {
                    stepsTaken++
                    consecutiveSteps++
                    val currentStepIntervalMs = if (timeSinceLastStep > 0L) (currentTime - timeSinceLastStep) else 550L
                    timeSinceLastStep = currentTime
                    
                    val currentVx = eskf.velocity.x
                    val currentVy = eskf.velocity.y
                    
                    val heading = if (kotlin.math.abs(currentVx) > 0.1 || kotlin.math.abs(currentVy) > 0.1) {
                        kotlin.math.atan2(currentVy, currentVx)
                    } else {
                        kotlin.math.atan2(rotationMatrix[4].toDouble(), rotationMatrix[1].toDouble())
                    }
                    
                    // Strict Gating PDR Auto-calibration
                    val gnssVel = gnssAdapter.getVelocity()
                    if (gnssVel != null && gnssVel.covariance < 0.5 && stepDetector.isCadenceStable) {
                        val gnssMag = kotlin.math.sqrt(gnssVel.vx * gnssVel.vx + gnssVel.vy * gnssVel.vy)
                        if (gnssMag > 0.5) {
                            val gnssHeading = kotlin.math.atan2(gnssVel.vy, gnssVel.vx)
                            val headingDiff = kotlin.math.abs(gnssHeading - heading)
                            if (headingDiff < 0.3 || headingDiff > 2 * Math.PI - 0.3) {
                                val dtStepSec = currentStepIntervalMs / 1000.0
                                val observedStepLength = gnssMag * dtStepSec
                                val baseModelLength = stepDetector.stepLength / stepDetector.stepK_multiplier
                                if (baseModelLength > 0.05) {
                                    val observedK = (observedStepLength / baseModelLength).toFloat()
                                    stepDetector.stepK_multiplier = stepDetector.stepK_multiplier * 0.95f + observedK * 0.05f
                                }
                            }
                        }
                    }

                    // Feed step velocity into ESKF directly without drop/buffering
                    val stepVel = stepDetector.stepVelocity
                    val vx_pdr = stepVel * kotlin.math.cos(heading)
                    val vy_pdr = stepVel * kotlin.math.sin(heading)
                    val stepVelArray = doubleArrayOf(vx_pdr, vy_pdr, 0.0)
                    
                    eskf.updateVelocity(stepVelArray, 0.1)
                    
                    val logMsg = String.format("Step %d (Seq: %d): L=%.2fm, Vel=%.2fm/s, H=%.1f°, K=%.2f", 
                        stepsTaken, consecutiveSteps, stepDetector.stepLength, stepVel, heading * 180.0 / Math.PI, stepDetector.stepK_multiplier)
                    sendBroadcast(Intent("com.example.inertialgps.PDR_LOG").apply {
                        setPackage(packageName)
                        putExtra("log", logMsg)
                    })
                }

                // STATE 3: NHC (Transport Mode)
                if (!isZuptActive && (currentTime - timeSinceLastStep > 5000)) {
                    val gyroMag = kotlin.math.sqrt(currentGyro[0]*currentGyro[0] + currentGyro[1]*currentGyro[1] + currentGyro[2]*currentGyro[2])
                    
                    // Rotation Watchdog: Only apply NHC if phone is not spinning in hands
                    if (gyroMag < 0.3f) { 
                        val currentVx = eskf.velocity.x
                        val currentVy = eskf.velocity.y
                        
                        val gnssVel = gnssAdapter.getVelocity()
                        val (vX, vY, vMag) = if (gnssVel != null && gnssVel.covariance < 0.5) {
                            val mag = kotlin.math.sqrt(gnssVel.vx * gnssVel.vx + gnssVel.vy * gnssVel.vy)
                            Triple(gnssVel.vx, gnssVel.vy, mag)
                        } else {
                            val mag = kotlin.math.sqrt(currentVx * currentVx + currentVy * currentVy)
                            Triple(currentVx, currentVy, mag)
                        }
                        
                        currentNhcHeading = 0.0
                        var applyNhc = false

                        if (vMag > 2.0) {
                            currentNhcHeading = kotlin.math.atan2(vY, vX)
                            // Save world-to-body heading
                            val wX = kotlin.math.cos(currentNhcHeading).toFloat()
                            val wY = kotlin.math.sin(currentNhcHeading).toFloat()
                            
                            savedBodyAxisX = (rotationMatrix[0]*wX + rotationMatrix[3]*wY + rotationMatrix[6]*0f).toDouble()
                            savedBodyAxisY = (rotationMatrix[1]*wX + rotationMatrix[4]*wY + rotationMatrix[7]*0f).toDouble()
                            hasSavedBodyAxis = true
                            applyNhc = true
                        } else if (hasSavedBodyAxis) {
                            val bX = savedBodyAxisX.toFloat()
                            val bY = savedBodyAxisY.toFloat()
                            
                            val wX = rotationMatrix[0]*bX + rotationMatrix[1]*bY
                            val wY = rotationMatrix[3]*bX + rotationMatrix[4]*bY
                            
                            currentNhcHeading = kotlin.math.atan2(wY.toDouble(), wX.toDouble())
                            applyNhc = true
                        }
                        
                        if (applyNhc) {
                            val rCov = if (vMag > 2.0) 0.1 else 1.0 // Soft NHC on low speeds
                            eskf.updateLateralVelocity(currentNhcHeading, rCov)
                        }
                    } else {
                        hasSavedBodyAxis = false // Reset freeze if phone turned heavily
                    }
                }
                
                // STATE 4: Pure INS 
                // Happens automatically 200 times a second via eskf.predict() when no updates are applied.
                
                // ==========================================
                
                // Throttle UI and Mock Location updates to prevent Broadcast flooding (1 Hz or on Step)
                if (currentTime - lastMockUpdateTime > 1000 || isStep) {
                    if (currentTime - lastMockUpdateTime > 1000) {
                        val v = eskf.velocity
                        val p = eskf.position
                        val b = eskf.accelBias
                        val zuptStr = if (isZuptActive) "ZUPT Active" else "Moving"
                        val sysLog = String.format("Pos: %.2f, %.2f, %.2f\nVel: %.2f, %.2f, %.2f\nBias: %.3f, %.3f, %.3f\nState: %s", 
                            p.x, p.y, p.z,
                            v.x, v.y, v.z,
                            b.x, b.y, b.z,
                            zuptStr)
                        sendBroadcast(Intent("com.example.inertialgps.SYS_LOG").apply {
                            setPackage(packageName)
                            putExtra("log", sysLog)
                        })
                        
                        // Detailed diagnostics
                        val vGnss = gnssAdapter.getVelocity()
                        val timeSinceFusion = if (lastGnssFusionTime > 0L) (currentTime - lastGnssFusionTime) / 1000 else -1L
                        val fusionStatus = if (timeSinceFusion in 0L..3L) {
                            String.format("Active (%ds ago, count: %d)", timeSinceFusion, gnssFusionCount)
                        } else {
                            String.format("Inactive (%s)", if (timeSinceFusion >= 0) "${timeSinceFusion}s ago" else "never")
                        }

                        val hVar = kotlin.math.max(eskf.P[0] + eskf.P[10], 0.0)
                        val filterAcc1Sigma = kotlin.math.sqrt(hVar)

                        val diagLog = StringBuilder()
                        diagLog.append("--- ENGINE DIAGNOSTICS ---\n")
                        diagLog.append(String.format("ZUPT State: %s\n", zuptStr))
                        diagLog.append(String.format("GNSS Fusion: %s\n", fusionStatus))
                        diagLog.append(String.format("GNSS Velocity: %s\n", if (vGnss != null) String.format("%.2f, %.2f, %.2f (cov: %.2f)", vGnss.vx, vGnss.vy, vGnss.vz, vGnss.covariance) else "Unavailable"))
                        diagLog.append(String.format("Filter Accuracy (1σ): %.2f m\n", filterAcc1Sigma))
                        val baroStr = if (isBarometerAvailable) {
                            String.format("%.1f hPa (ΔAlt: %.2fm)", currentPressure, filteredBaroAlt)
                        } else {
                            "Unavailable (No sensor)"
                        }
                        diagLog.append(String.format("Barometer: %s\n", baroStr))
                        diagLog.append(String.format("NHC Freeze Active: %s\n", hasSavedBodyAxis))
                        if (hasSavedBodyAxis) {
                            diagLog.append(String.format("NHC Body X: %.2f, Y: %.2f\n", savedBodyAxisX, savedBodyAxisY))
                            diagLog.append(String.format("NHC Heading (rad): %.2f\n", currentNhcHeading))
                        }
                        diagLog.append(String.format("ESKF Velocity (Mag): %.2f m/s\n", kotlin.math.sqrt(v.x*v.x + v.y*v.y)))
                        
                        sendBroadcast(Intent("com.example.inertialgps.DIAG_LOG").apply {
                            setPackage(packageName)
                            putExtra("log", diagLog.toString())
                        })
                        
                        lastMockUpdateTime = currentTime
                    }
                    
                    positionOffset[0] = eskf.position.x.toFloat()
                    positionOffset[1] = eskf.position.y.toFloat()
                    positionOffset[2] = eskf.position.z.toFloat()

                    velocity[0] = eskf.velocity.x.toFloat()
                    velocity[1] = eskf.velocity.y.toFloat()
                    velocity[2] = eskf.velocity.z.toFloat()
                    
                    updateMockLocation()
                }
                
            } catch (e: Exception) {
                Log.e("MockLocationService", "Sensor processing exception", e)
            }
        }
    }

    private fun updateMockLocation() {
        // High-precision WGS-84 Geodetic ENU->ECEF->LLA transformation
        Wgs84Converter.enuToLla(
            refLatDeg = initialLat,
            refLonDeg = initialLon,
            refAltMeters = initialAlt,
            eastMeters = positionOffset[0].toDouble(),
            northMeters = positionOffset[1].toDouble(),
            upMeters = positionOffset[2].toDouble(),
            outLla = currentLla
        )
        val newLat = currentLla[0]
        val newLon = currentLla[1]
        val newAlt = currentLla[2]

        val currentSpeed = kotlin.math.sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1])
        val currentBearing = ((kotlin.math.atan2(velocity[0], velocity[1]) * 180 / PI + 360) % 360).toFloat()

        // Calculate dynamic accuracy from ESKF covariance (P_xx + P_yy) with 1.5 multiplier (approx 85% confidence)
        val hVar = kotlin.math.max(eskf.P[0] + eskf.P[10], 0.0)
        val estimatedAccuracy = (kotlin.math.sqrt(hVar) * 1.5).toFloat().coerceIn(1.0f, 50.0f)

        val baseLocation = Location(LocationManager.GPS_PROVIDER).apply {
            latitude = newLat
            longitude = newLon
            altitude = newAlt
            speed = currentSpeed
            bearing = currentBearing
            time = System.currentTimeMillis()
            elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
            accuracy = estimatedAccuracy
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
        unregisterSensors()
        disableInertialMode()
        sensorThread?.quitSafely()
        sensorThread = null
        sensorHandler = null
        sendBroadcast(Intent("com.example.inertialgps.SERVICE_STOPPED").apply { setPackage(packageName) })
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
