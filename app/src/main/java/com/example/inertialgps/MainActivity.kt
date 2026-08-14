package com.example.inertialgps

import android.Manifest
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.GravityCompat
import com.example.inertialgps.databinding.ActivityMainBinding
import java.io.PrintWriter
import java.io.StringWriter

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

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
                    binding.tvLocation.text = String.format("Lat: %.6f, Lon: %.6f", lastLat, lastLon)
                }
                "com.example.inertialgps.GPS_WAITING" -> {
                    binding.tvLocation.text = "Waiting for Real GPS fix..."
                }
                "com.example.inertialgps.MOCK_DENIED" -> {
                    Toast.makeText(this@MainActivity, "Mock Location not set in Developer Options!", Toast.LENGTH_LONG).show()
                    stopMockService()
                }
                "com.example.inertialgps.SERVICE_STOPPED" -> {
                    isServiceRunning = false
                    isInertialEnabled = false
                    updateUIState()
                }
                "com.example.inertialgps.PDR_LOG" -> {
                    val log = intent.getStringExtra("log") ?: return
                    val currentText = binding.tvLogs.text.toString()
                    val lines = currentText.split("\n").takeLast(10)
                    binding.tvLogs.text = lines.joinToString("\n") + "\n" + log
                }
                "com.example.inertialgps.SYS_LOG" -> {
                    val log = intent.getStringExtra("log") ?: return
                    binding.tvSysLogs.text = "System Status:\n$log"
                }
                "com.example.inertialgps.DIAG_LOG" -> {
                    val log = intent.getStringExtra("log") ?: return
                    binding.tvDiagnostics.text = log
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.topAppBar.setNavigationOnClickListener {
            binding.drawerLayout.openDrawer(GravityCompat.START)
        }
        
        binding.navView.setNavigationItemSelectedListener { menuItem ->
            when (menuItem.itemId) {
                R.id.nav_panel -> {
                    binding.layoutPanel.visibility = View.VISIBLE
                    binding.layoutLogs.visibility = View.GONE
                    binding.topAppBar.title = "Control Panel"
                }
                R.id.nav_logs -> {
                    binding.layoutPanel.visibility = View.GONE
                    binding.layoutLogs.visibility = View.VISIBLE
                    binding.topAppBar.title = "Diagnostic Logs"
                }
            }
            binding.drawerLayout.closeDrawer(GravityCompat.START)
            true
        }

        binding.btnShowMap.setOnClickListener {
            if (lastLat != 0.0 && lastLon != 0.0) {
                val uri = Uri.parse("geo:$lastLat,$lastLon?q=$lastLat,$lastLon(InertialGPS Mock)")
                val mapIntent = Intent(Intent.ACTION_VIEW, uri).apply {
                    setPackage("com.google.android.apps.maps")
                }
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
            binding.tvStatus.text = "CRASH: $crashLog"
            prefs.edit().remove("crash_log").apply()
        }

        val defaultHandler = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
            val sw = StringWriter()
            throwable.printStackTrace(PrintWriter(sw))
            prefs.edit().putString("crash_log", sw.toString()).apply()
            defaultHandler?.uncaughtException(thread, throwable)
        }
        isServiceRunning = prefs.getBoolean("isServiceRunning", false)
        isInertialEnabled = prefs.getBoolean("isInertialEnabled", false)
        updateUIState()

        binding.btnToggle.setOnClickListener {
            if (isServiceRunning) {
                stopMockService()
            } else {
                checkPermissionsAndStart()
            }
        }

        binding.btnToggleInertial.setOnClickListener {
            isInertialEnabled = !isInertialEnabled
            val intent = Intent(this, MockLocationService::class.java).apply {
                action = if (isInertialEnabled) "ENABLE_INERTIAL" else "DISABLE_INERTIAL"
            }
            startService(intent)
            updateUIState()
        }

        binding.btnShareLogs.setOnClickListener {
            val diagText = binding.tvDiagnostics.text.toString()
            val sysText = binding.tvSysLogs.text.toString()
            val pdrText = binding.tvLogs.text.toString()
            val fullReport = StringBuilder().apply {
                append("=== InertialGPS Diagnostic Report ===\n\n")
                append(diagText).append("\n\n")
                append(sysText).append("\n\n")
                append(pdrText)
            }.toString()

            val shareIntent = Intent(Intent.ACTION_SEND).apply {
                type = "text/plain"
                putExtra(Intent.EXTRA_SUBJECT, "InertialGPS Diagnostics")
                putExtra(Intent.EXTRA_TEXT, fullReport)
            }
            startActivity(Intent.createChooser(shareIntent, "Share Diagnostic Logs"))
        }

        binding.btnClearLogs.setOnClickListener {
            binding.tvLogs.text = "PDR Logs:\n"
            Toast.makeText(this, "Logs cleared", Toast.LENGTH_SHORT).show()
        }
    }

    override fun onResume() {
        super.onResume()
        val prefs = getSharedPreferences("InertialGPS", Context.MODE_PRIVATE)
        isServiceRunning = prefs.getBoolean("isServiceRunning", false)
        isInertialEnabled = prefs.getBoolean("isInertialEnabled", false)
        updateUIState()

        val filter = IntentFilter().apply {
            addAction("com.example.inertialgps.LOCATION_UPDATE")
            addAction("com.example.inertialgps.GPS_WAITING")
            addAction("com.example.inertialgps.MOCK_DENIED")
            addAction("com.example.inertialgps.SERVICE_STOPPED")
            addAction("com.example.inertialgps.PDR_LOG")
            addAction("com.example.inertialgps.SYS_LOG")
            addAction("com.example.inertialgps.DIAG_LOG")
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
            binding.btnToggle.text = "Stop Service"
            binding.btnToggleInertial.isEnabled = true
        } else {
            binding.btnToggle.text = "Start Service"
            binding.btnToggleInertial.isEnabled = false
            isInertialEnabled = false
        }
        
        if (isInertialEnabled) {
            binding.btnToggleInertial.text = "Disable Inertial Mode"
            binding.tvStatus.text = "Status: Service Running (Inertial ON)"
        } else {
            binding.btnToggleInertial.text = "Enable Inertial Mode"
            binding.tvStatus.text = if (isServiceRunning) "Status: Service Running" else "Status: Stopped"
        }
    }

    private fun startMockService() {
        val intent = Intent(this, MockLocationService::class.java).apply {
            action = "START_SERVICE"
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
        isServiceRunning = true
        updateUIState()
    }

    private fun stopMockService() {
        val intent = Intent(this, MockLocationService::class.java).apply {
            action = "STOP_SERVICE"
        }
        startService(intent)
        isServiceRunning = false
        isInertialEnabled = false
        updateUIState()
    }
}
