package com.example.inertialgps.gnss

/**
 * Interface representing a mathematical solver that converts raw GNSS measurements 
 * into a Cartesian velocity vector (ENU or World Frame).
 * This isolates the ESKF Core from any specific 3rd-party PVT library.
 */
interface GnssVelocityProvider {
    /**
     * @return A data class containing the 3D velocity [Vx, Vy, Vz] and its covariance matrix/scalar,
     * or null if the solver hasn't converged or doesn't have enough satellites.
     */
    fun getVelocity(): GnssVelocity?

    /**
     * Start listening to the hardware and computing velocity.
     */
    fun start()

    /**
     * Stop hardware listeners to save battery.
     */
    fun stop()
}

data class GnssVelocity(
    val vx: Double,
    val vy: Double,
    val vz: Double,
    val covariance: Double // Simplified scalar covariance (e.g., based on DOP and residuals)
)
