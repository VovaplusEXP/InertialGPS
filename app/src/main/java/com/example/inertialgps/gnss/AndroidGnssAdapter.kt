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
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

/**
 * Adapter that listens to raw Android GNSS hardware measurements and passes them to the PVT Solver.
 * Utilizes the API 31+ GnssMeasurementRequest to force full tracking (disable duty cycling).
 */
class AndroidGnssAdapter(private val context: Context) : GnssVelocityProvider {

    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private var executor: ExecutorService? = null
    
    // Instantiate the imported Google WLS Solver
    private val pvtSolver = PseudorangePositionVelocityFromRealTimeEvents()
    
    // The currently computed velocity from the PVT Solver
    @Volatile private var currentVelocity: GnssVelocity? = null
    @Volatile private var onVelocityListener: ((GnssVelocity) -> Unit)? = null
    @Volatile private var referenceSet = false

    private val gnssCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            try {
                if (!referenceSet) {
                    val lastLoc = try {
                        locationManager.getLastKnownLocation(LocationManager.PASSIVE_PROVIDER)
                            ?: locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER)
                    } catch (e: SecurityException) {
                        null
                    }
                    if (lastLoc != null) {
                        setReferencePosition(lastLoc.latitude, lastLoc.longitude, lastLoc.altitude)
                    }
                }

                // Feed raw measurements to the Google WLS solver
                pvtSolver.computePositionVelocitySolutionsFromRawMeas(event)
                
                val velEnu = pvtSolver.velocitySolutionEnuMps
                val uncertEnu = pvtSolver.positionVelocityUncertaintyEnu
                
                // Check if solver successfully output valid numbers (not NaN)
                if (velEnu != null && velEnu.size >= 3 && !velEnu[0].isNaN() && !velEnu[1].isNaN() && !velEnu[2].isNaN()) {
                    // Extract Covariance/Uncertainty (using max horizontal uncertainty as a scalar)
                    var covariance = 100.0
                    if (uncertEnu != null && uncertEnu.size >= 6 && !uncertEnu[3].isNaN()) {
                        covariance = kotlin.math.max(uncertEnu[3], uncertEnu[4])
                    }
                    
                    val velocity = GnssVelocity(
                        vx = velEnu[0],
                        vy = velEnu[1],
                        vz = velEnu[2],
                        covariance = covariance
                    )
                    currentVelocity = velocity
                    onVelocityListener?.invoke(velocity)
                } else {
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

    override fun setOnVelocityListener(listener: ((GnssVelocity) -> Unit)?) {
        this.onVelocityListener = listener
    }

    override fun setReferencePosition(lat: Double, lon: Double, alt: Double) {
        referenceSet = true
        pvtSolver.setReferencePosition(
            (lat * 1e7).toInt(),
            (lon * 1e7).toInt(),
            (alt * 1e7).toInt()
        )
    }

    override fun getVelocity(): GnssVelocity? {
        return currentVelocity
    }

    override fun start() {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            Log.e("GNSS_Adapter", "Missing ACCESS_FINE_LOCATION permission.")
            return
        }

        if (executor == null || executor!!.isShutdown) {
            executor = Executors.newSingleThreadExecutor()
        }
        val currentExec = executor!!

        try {
            // Register Navigation Messages
            locationManager.registerGnssNavigationMessageCallback(currentExec, navCallback)

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                // API 31+: Force Full Tracking to bypass Duty Cycling
                val request = GnssMeasurementRequest.Builder()
                    .setFullTracking(true)
                    .build()
                locationManager.registerGnssMeasurementsCallback(request, currentExec, gnssCallback)
            } else {
                locationManager.registerGnssMeasurementsCallback(gnssCallback)
            }
            Log.d("GNSS_Adapter", "Successfully registered for raw GNSS measurements.")
        } catch (e: Exception) {
            Log.e("GNSS_Adapter", "Failed to register GNSS callback", e)
        }
    }

    override fun stop() {
        try {
            locationManager.unregisterGnssNavigationMessageCallback(navCallback)
            locationManager.unregisterGnssMeasurementsCallback(gnssCallback)
        } catch (e: Exception) {
            Log.e("GNSS_Adapter", "Error unregistering GNSS callbacks", e)
        }
        executor?.shutdown()
        executor = null
        currentVelocity = null
        Log.d("GNSS_Adapter", "Unregistered GNSS measurements.")
    }
}
