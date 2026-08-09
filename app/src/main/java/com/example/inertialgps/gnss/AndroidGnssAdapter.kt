package com.example.inertialgps.gnss

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.GnssMeasurementRequest
import android.location.GnssMeasurementsEvent
import android.location.LocationManager
import android.os.Build
import android.util.Log
import androidx.core.app.ActivityCompat
import java.util.concurrent.Executors

/**
 * Adapter that listens to raw Android GNSS hardware measurements and passes them to the PVT Solver.
 * Utilizes the API 31+ GnssMeasurementRequest to force full tracking (disable duty cycling).
 */
class AndroidGnssAdapter(private val context: Context) : GnssVelocityProvider {

    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private val executor = Executors.newSingleThreadExecutor()
    
    // The currently computed velocity from the PVT Solver
    private var currentVelocity: GnssVelocity? = null

    private val gnssCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            val measurements = event.measurements
            if (measurements.isEmpty()) return

            // TODO: Pass 'measurements' to the 3rd-party WLS PVT Solver (e.g. GNSS Compare).
            // The solver will extract 'pseudorangeRateMetersPerSecond' from each satellite,
            // calculate Ephemeris, and perform Least Squares to find True Velocity.
            
            // For now, we log the number of visible satellites providing Doppler data
            var dopplerCount = 0
            for (measurement in measurements) {
                if (measurement.pseudorangeRateState != 0) {
                    dopplerCount++
                }
            }
            // Log.d("GNSS_Adapter", "Received raw measurements from $dopplerCount satellites with Doppler.")
            
            // When the solver is implemented, it will set currentVelocity here:
            // currentVelocity = pvtSolver.solveVelocity(measurements)
        }

        override fun onStatusChanged(status: Int) {
            super.onStatusChanged(status)
            Log.d("GNSS_Adapter", "GNSS Status changed: $status")
        }
    }

    override fun getVelocity(): GnssVelocity? {
        return currentVelocity
    }

    override fun start() {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            Log.e("GNSS_Adapter", "Missing ACCESS_FINE_LOCATION permission.")
            return
        }

        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                // API 31+: Force Full Tracking to bypass Duty Cycling
                val request = GnssMeasurementRequest.Builder()
                    .setFullTracking(true)
                    .build()
                locationManager.registerGnssMeasurementsCallback(request, executor, gnssCallback)
            } else {
                // API < 31: Legacy registration (may suffer from duty cycling if not disabled in dev options)
                locationManager.registerGnssMeasurementsCallback(gnssCallback)
            }
            Log.d("GNSS_Adapter", "Successfully registered for raw GNSS measurements.")
        } catch (e: Exception) {
            Log.e("GNSS_Adapter", "Failed to register GNSS callback", e)
        }
    }

    override fun stop() {
        locationManager.unregisterGnssMeasurementsCallback(gnssCallback)
        Log.d("GNSS_Adapter", "Unregistered GNSS measurements.")
    }
}
