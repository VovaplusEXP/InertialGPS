package com.example.inertialgps

import android.Manifest
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import android.net.Uri
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

class MainActivity : AppCompatActivity() {

    private lateinit var tvStatus: TextView
    private lateinit var tvLocation: TextView
    private lateinit var btnToggle: Button
    private lateinit var btnToggleInertial: Button
    private lateinit var btnShowMap: Button
    private lateinit var tvSysLogs: TextView
    private lateinit var tvLogs: TextView

    private var isServiceRunning = false
    private var isInertialEnabled = false
    private var lastLat = 0.0
    private var lastLon = 0.0

    private val locationUpdateReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                "com.example.inertialgps.LOCATION_UPDATE" -> {
                    lastLat = intent.getDoubleExtra("lat", 0.0)
                    lastLon = intent.getDoubleExtra("lon", 0.0)
                    tvLocation.text = String.format("Lat: %.6f, Lon: %.6f", lastLat, lastLon)
                }
                "com.example.inertialgps.GPS_WAITING" -> {
                    tvLocation.text = "Waiting for Real GPS fix..."
                }
                "com.example.inertialgps.MOCK_DENIED" -> {
                    Toast.makeText(this@MainActivity, "Mock Location not set in Developer Options!", Toast.LENGTH_LONG).show()
                    stopMockService()
                }
                "com.example.inertialgps.PDR_LOG" -> {
                    val log = intent.getStringExtra("log") ?: return
                    val currentText = tvLogs.text.toString()
                    val lines = currentText.split("\n").takeLast(10) // Keep last 10 lines
                    tvLogs.text = lines.joinToString("\n") + "\n" + log
                }
                "com.example.inertialgps.SYS_LOG" -> {
                    val log = intent.getStringExtra("log") ?: return
                    tvSysLogs.text = "System Status:\n$log"
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        tvStatus = findViewById(R.id.tvStatus)
        tvLocation = findViewById(R.id.tvLocation)
        btnToggle = findViewById(R.id.btnToggle)
        btnToggleInertial = findViewById(R.id.btnToggleInertial)
        btnShowMap = findViewById(R.id.btnShowMap)
        tvSysLogs = findViewById(R.id.tvSysLogs)
        tvLogs = findViewById(R.id.tvLogs)
        
        btnShowMap.setOnClickListener {
            if (lastLat != 0.0 && lastLon != 0.0) {
                val uri = Uri.parse("geo:$lastLat,$lastLon?q=$lastLat,$lastLon(InertialGPS Mock)")
                val mapIntent = Intent(Intent.ACTION_VIEW, uri)
                mapIntent.setPackage("com.google.android.apps.maps")
                if (mapIntent.resolveActivity(packageManager) != null) {
                    startActivity(mapIntent)
                } else {
                    val genericIntent = Intent(Intent.ACTION_VIEW, uri)
                    startActivity(genericIntent)
                }
            } else {
                Toast.makeText(this, "No location yet", Toast.LENGTH_SHORT).show()
            }
        }

        val prefs = getSharedPreferences("InertialGPS", Context.MODE_PRIVATE)
        
        val crashLog = prefs.getString("crash_log", null)
        if (crashLog != null) {
            tvStatus.text = "CRASH: $crashLog"
            prefs.edit().remove("crash_log").commit()
        }

        val defaultHandler = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
            val sw = java.io.StringWriter()
            throwable.printStackTrace(java.io.PrintWriter(sw))
            prefs.edit().putString("crash_log", sw.toString()).commit()
            defaultHandler?.uncaughtException(thread, throwable)
        }
        isServiceRunning = prefs.getBoolean("isServiceRunning", false)
        isInertialEnabled = prefs.getBoolean("isInertialEnabled", false)

        val bx = prefs.getFloat("biasX", 0f)
        val by = prefs.getFloat("biasY", 0f)
        val bz = prefs.getFloat("biasZ", 0f)
        if (bx != 0f || by != 0f || bz != 0f) {
            tvCalibration.text = String.format("Bias: X:%.3f Y:%.3f Z:%.3f", bx, by, bz)
        }
        
        updateUIState()

        btnToggle.setOnClickListener {
            if (isServiceRunning) {
                stopMockService()
            } else {
                checkPermissionsAndStart()
            }
        }

        btnToggleInertial.setOnClickListener {
            isInertialEnabled = !isInertialEnabled
            val intent = Intent(this, MockLocationService::class.java)
            if (isInertialEnabled) {
                intent.action = "ENABLE_INERTIAL"
            } else {
                intent.action = "DISABLE_INERTIAL"
            }
            startService(intent)
            updateUIState()
        }
    }

    override fun onResume() {
        super.onResume()
        val filter = IntentFilter().apply {
            addAction("com.example.inertialgps.LOCATION_UPDATE")
            addAction("com.example.inertialgps.GPS_WAITING")
            addAction("com.example.inertialgps.MOCK_DENIED")
            addAction("com.example.inertialgps.PDR_LOG")
            addAction("com.example.inertialgps.SYS_LOG")
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(locationUpdateReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
        } else {
            registerReceiver(locationUpdateReceiver, filter)
        }
    }

    override fun onPause() {
        super.onPause()
        unregisterReceiver(locationUpdateReceiver)
    }

    private fun checkPermissionsAndStart() {
        val permissions = mutableListOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions.add(Manifest.permission.POST_NOTIFICATIONS)
        }
        val needed = permissions.filter { ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED }
        if (needed.isNotEmpty()) {
            ActivityCompat.requestPermissions(this, needed.toTypedArray(), 100)
        } else {
            startMockService()
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == 100 && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
            startMockService()
        }
    }

    private fun updateUIState() {
        if (isServiceRunning) {
            btnToggle.text = "Stop Service"
            btnToggleInertial.isEnabled = true
        } else {
            btnToggle.text = "Start Service"
            btnToggleInertial.isEnabled = false
            isInertialEnabled = false
        }
        
        if (isInertialEnabled) {
            btnToggleInertial.text = "Disable Inertial Mode"
            tvStatus.text = "Status: Service Running (Inertial ON)"
        } else {
            btnToggleInertial.text = "Enable Inertial Mode"
            tvStatus.text = if (isServiceRunning) "Status: Service Running" else "Status: Stopped"
        }
    }

    private fun startMockService() {
        val intent = Intent(this, MockLocationService::class.java)
        intent.action = "START_SERVICE"
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
        isServiceRunning = true
        updateUIState()
    }

    private fun stopMockService() {
        val intent = Intent(this, MockLocationService::class.java)
        stopService(intent)
        isServiceRunning = false
        isInertialEnabled = false
        updateUIState()
    }
}
