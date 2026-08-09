package com.example.inertialgps.gnss

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.GnssMeasurementRequest
import android.location.GnssMeasurementsEvent
import android.location.GnssNavigationMessage
import android.location.LocationManager
import android.os.Build
import android.util.Log
import androidx.core.app.ActivityCompat
import com.google.location.lbs.gnss.gps.pseudorange.PseudorangePositionVelocityFromRealTimeEvents
import java.util.concurrent.Executors

/**
 * Adapter that listens to raw Android GNSS hardware measurements and passes them to the PVT Solver.
 * Utilizes the API 31+ GnssMeasurementRequest to force full tracking (disable duty cycling).
 */
class AndroidGnssAdapter(private val context: Context) : GnssVelocityProvider {

    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private val executor = Executors.newSingleThreadExecutor()
    
    // Instantiate the imported Google WLS Solver
    private val pvtSolver = PseudorangePositionVelocityFromRealTimeEvents()
    
    // The currently computed velocity from the PVT Solver
    private var currentVelocity: GnssVelocity? = null

    private val gnssCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            val measurements = event.measurements
            try {
                // Ensure the solver has a reference location to initialize properly (or else it aborts)
                val lastLoc = locationManager.getLastKnownLocation(LocationManager.PASSIVE_PROVIDER)
                if (lastLoc != null) {
                    pvtSolver.setReferencePosition(
                        (lastLoc.latitude * 1e7).toInt(), 
                        (lastLoc.longitude * 1e7).toInt(), 
                        (lastLoc.altitude * 1e7).toInt()
                    )
                } else {
                    pvtSolver.setReferencePosition(0, 0, 0)
                }

                // Feed raw measurements to the Google WLS solver
                pvtSolver.computePositionVelocitySolutionsFromRawMeas(event)
                
                val velEnu = pvtSolver.velocitySolutionEnuMps
                val uncertEnu = pvtSolver.positionVelocityUncertaintyEnu
                
                // Check if solver successfully output valid numbers (not NaN)
                if (velEnu != null && velEnu.size >= 3 && !velEnu[0].isNaN()) {
                    // Extract Covariance/Uncertainty (using max horizontal uncertainty as a scalar)
                    var covariance = 100.0
                    if (uncertEnu != null && uncertEnu.size >= 6 && !uncertEnu[3].isNaN()) {
                        // Indexes 3,4 are Vx, Vy uncertainties
                        covariance = kotlin.math.max(uncertEnu[3], uncertEnu[4])
                    }
                    
                    currentVelocity = GnssVelocity(
                        vx = velEnu[0],
                        vy = velEnu[1],
                        vz = velEnu[2],
                        covariance = covariance
                    )
                } else {
                    // Solver didn't converge this epoch
                    currentVelocity = null
                }
            } catch (e: Exception) {
                Log.e("GNSS_Adapter", "PVT Solver Error", e)
                currentVelocity = null
            }
        }

        override fun onStatusChanged(status: Int) {
            super.onStatusChanged(status)
            Log.d("GNSS_Adapter", "GNSS Status changed: $status")
        }
    }
    
    private val navCallback = object : GnssNavigationMessage.Callback() {
        override fun onGnssNavigationMessageReceived(event: GnssNavigationMessage) {
            try {
                pvtSolver.parseHwNavigationMessageUpdates(event)
            } catch (e: Exception) {
                Log.e("GNSS_Adapter", "Nav Message Parse Error", e)
            }
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
            // Register Navigation Messages
            locationManager.registerGnssNavigationMessageCallback(executor, navCallback)

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
        locationManager.unregisterGnssNavigationMessageCallback(navCallback)
        locationManager.unregisterGnssMeasurementsCallback(gnssCallback)
        Log.d("GNSS_Adapter", "Unregistered GNSS measurements.")
    }
}
