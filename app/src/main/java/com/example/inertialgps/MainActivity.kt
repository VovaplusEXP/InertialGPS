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
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

class MainActivity : AppCompatActivity() {

    private lateinit var tvStatus: TextView
    private lateinit var tvLocation: TextView
    private lateinit var btnToggle: Button
    private lateinit var btnToggleInertial: Button

    private var isServiceRunning = false
    private var isInertialEnabled = false

    private val locationUpdateReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent?.action == "LOCATION_UPDATE") {
                val lat = intent.getDoubleExtra("lat", 0.0)
                val lon = intent.getDoubleExtra("lon", 0.0)
                tvLocation.text = String.format("Lat: %.6f, Lon: %.6f", lat, lon)
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
                btnToggleInertial.text = "Disable Inertial Mode"
                tvStatus.text = "Status: Service Running (Inertial ON)"
            } else {
                intent.action = "DISABLE_INERTIAL"
                btnToggleInertial.text = "Enable Inertial Mode"
                tvStatus.text = "Status: Service Running (Inertial OFF)"
            }
            startService(intent) // Send intent to running service
        }
    }

    override fun onResume() {
        super.onResume()
        registerReceiver(locationUpdateReceiver, IntentFilter("LOCATION_UPDATE"))
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

    private fun startMockService() {
        val intent = Intent(this, MockLocationService::class.java)
        intent.action = "START_SERVICE"
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
        isServiceRunning = true
        btnToggle.text = "Stop Service"
        btnToggleInertial.isEnabled = true
        tvStatus.text = "Status: Service Running (Inertial OFF)"
    }

    private fun stopMockService() {
        val intent = Intent(this, MockLocationService::class.java)
        stopService(intent)
        isServiceRunning = false
        isInertialEnabled = false
        btnToggle.text = "Start Service"
        btnToggleInertial.isEnabled = false
        btnToggleInertial.text = "Enable Inertial Mode"
        tvStatus.text = "Status: Stopped"
    }
}
