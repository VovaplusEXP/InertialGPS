package com.example.inertialgps

import kotlin.math.*

/**
 * High-precision WGS-84 Geodetic Converter.
 * Converts local East-North-Up (ENU) metric displacements relative to a reference
 * WGS-84 coordinate (lat, lon, alt) into exact geodetic latitude, longitude, and ellipsoidal altitude.
 * Uses Bowring's closed-form algorithm (accurate to sub-millimeter level everywhere on Earth).
 */
object Wgs84Converter {

    // WGS-84 Ellipsoid Constants
    const val A = 6378137.0 // Semi-major axis in meters
    const val F = 1.0 / 298.257223563 // Flattening
    const val B = A * (1.0 - F) // Semi-minor axis (~6356752.314245 m)
    const val E2 = 2.0 * F - F * F // First eccentricity squared e^2 (~6.69437999014e-3)
    const val E_PRIME_2 = (A * A - B * B) / (B * B) // Second eccentricity squared e'^2 (~6.73949674228e-3)

    /**
     * Converts LLA (latDeg, lonDeg, altMeters) to ECEF (x, y, z in meters).
     */
    fun llaToEcef(latDeg: Double, lonDeg: Double, altMeters: Double, outEcef: DoubleArray) {
        val latRad = Math.toRadians(latDeg)
        val lonRad = Math.toRadians(lonDeg)

        val sinLat = sin(latRad)
        val cosLat = cos(latRad)
        val sinLon = sin(lonRad)
        val cosLon = cos(lonRad)

        val n = A / sqrt(1.0 - E2 * sinLat * sinLat)

        outEcef[0] = (n + altMeters) * cosLat * cosLon
        outEcef[1] = (n + altMeters) * cosLat * sinLon
        outEcef[2] = (n * (1.0 - E2) + altMeters) * sinLat
    }

    /**
     * Converts ECEF (x, y, z in meters) to LLA (latDeg, lonDeg, altMeters) using Bowring's closed-form.
     */
    fun ecefToLla(x: Double, y: Double, z: Double, outLla: DoubleArray) {
        val p = sqrt(x * x + y * y)
        if (p < 1e-6) {
            // Polar singularity check
            val lat = if (z >= 0) 90.0 else -90.0
            outLla[0] = lat
            outLla[1] = 0.0
            outLla[2] = abs(z) - B
            return
        }

        val theta = atan2(A * z, B * p)
        val sinTheta = sin(theta)
        val cosTheta = cos(theta)

        val latRad = atan2(
            z + E_PRIME_2 * B * sinTheta * sinTheta * sinTheta,
            p - E2 * A * cosTheta * cosTheta * cosTheta
        )
        val lonRad = atan2(y, x)

        val sinLat = sin(latRad)
        val cosLat = cos(latRad)
        val n = A / sqrt(1.0 - E2 * sinLat * sinLat)
        val altMeters = (p / cosLat) - n

        outLla[0] = Math.toDegrees(latRad)
        outLla[1] = Math.toDegrees(lonRad)
        outLla[2] = altMeters
    }

    /**
     * Converts local ENU displacement (east, north, up) relative to reference point (refLatDeg, refLonDeg, refAltMeters)
     * directly to target WGS-84 LLA (latDeg, lonDeg, altMeters).
     */
    fun enuToLla(
        refLatDeg: Double,
        refLonDeg: Double,
        refAltMeters: Double,
        eastMeters: Double,
        northMeters: Double,
        upMeters: Double,
        outLla: DoubleArray
    ) {
        val refLatRad = Math.toRadians(refLatDeg)
        val refLonRad = Math.toRadians(refLonDeg)

        val sinLat = sin(refLatRad)
        val cosLat = cos(refLatRad)
        val sinLon = sin(refLonRad)
        val cosLon = cos(refLonRad)

        // Reference point in ECEF
        val n0 = A / sqrt(1.0 - E2 * sinLat * sinLat)
        val x0 = (n0 + refAltMeters) * cosLat * cosLon
        val y0 = (n0 + refAltMeters) * cosLat * sinLon
        val z0 = (n0 * (1.0 - E2) + refAltMeters) * sinLat

        // Rotate ENU to ECEF displacement (Transpose of ECEF->ENU rotation matrix)
        val dx = -sinLon * eastMeters - sinLat * cosLon * northMeters + cosLat * cosLon * upMeters
        val dy = cosLon * eastMeters - sinLat * sinLon * northMeters + cosLat * sinLon * upMeters
        val dz = cosLat * northMeters + sinLat * upMeters

        // Convert target ECEF to LLA
        ecefToLla(x0 + dx, y0 + dy, z0 + dz, outLla)
    }
}
