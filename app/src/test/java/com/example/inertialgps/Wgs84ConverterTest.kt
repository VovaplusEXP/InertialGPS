package com.example.inertialgps

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class Wgs84ConverterTest {

    @Test
    fun testZeroDisplacement() {
        val refLat = 55.7558
        val refLon = 37.6173
        val refAlt = 156.0

        val outLla = DoubleArray(3)
        Wgs84Converter.enuToLla(
            refLatDeg = refLat,
            refLonDeg = refLon,
            refAltMeters = refAlt,
            eastMeters = 0.0,
            northMeters = 0.0,
            upMeters = 0.0,
            outLla = outLla
        )

        assertEquals(refLat, outLla[0], 1e-7)
        assertEquals(refLon, outLla[1], 1e-7)
        assertEquals(refAlt, outLla[2], 1e-4)
    }

    @Test
    fun testNorthDisplacement() {
        val refLat = 55.7558
        val refLon = 37.6173
        val refAlt = 150.0

        val outLla = DoubleArray(3)
        // 1000 meters North
        Wgs84Converter.enuToLla(
            refLatDeg = refLat,
            refLonDeg = refLon,
            refAltMeters = refAlt,
            eastMeters = 0.0,
            northMeters = 1000.0,
            upMeters = 0.0,
            outLla = outLla
        )

        // Latitude should increase by ~ 1000m / 111320m/deg approx 0.00898 deg
        assertTrue(outLla[0] > refLat)
        assertEquals(0.00898, outLla[0] - refLat, 0.001)
        // Longitude should remain virtually unchanged
        assertEquals(refLon, outLla[1], 1e-5)
    }

    @Test
    fun testEastDisplacement() {
        val refLat = 55.7558
        val refLon = 37.6173
        val refAlt = 150.0

        val outLla = DoubleArray(3)
        // 1000 meters East
        Wgs84Converter.enuToLla(
            refLatDeg = refLat,
            refLonDeg = refLon,
            refAltMeters = refAlt,
            eastMeters = 1000.0,
            northMeters = 0.0,
            upMeters = 0.0,
            outLla = outLla
        )

        // Longitude should increase
        assertTrue(outLla[1] > refLon)
        // Latitude should remain virtually unchanged
        assertEquals(refLat, outLla[0], 1e-5)
    }

    @Test
    fun testUpDisplacement() {
        val refLat = 0.0 // Equator
        val refLon = 0.0 // Prime meridian
        val refAlt = 100.0

        val outLla = DoubleArray(3)
        Wgs84Converter.enuToLla(
            refLatDeg = refLat,
            refLonDeg = refLon,
            refAltMeters = refAlt,
            eastMeters = 0.0,
            northMeters = 0.0,
            upMeters = 50.0,
            outLla = outLla
        )

        assertEquals(150.0, outLla[2], 1e-4)
    }

    @Test
    fun testLlaToEcefAndBackRoundtrip() {
        val lat = 48.8566
        val lon = 2.3522
        val alt = 35.0

        val ecef = DoubleArray(3)
        Wgs84Converter.llaToEcef(lat, lon, alt, ecef)

        val lla = DoubleArray(3)
        Wgs84Converter.ecefToLla(ecef[0], ecef[1], ecef[2], lla)

        assertEquals(lat, lla[0], 1e-7)
        assertEquals(lon, lla[1], 1e-7)
        assertEquals(alt, lla[2], 1e-4)
    }
}
