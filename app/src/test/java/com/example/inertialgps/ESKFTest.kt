package com.example.inertialgps

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import kotlin.math.abs

class ESKFTest {

    private lateinit var eskf: ESKF

    // Identity rotation matrix (row-major 3x3)
    private val identityR = floatArrayOf(
        1f, 0f, 0f,
        0f, 1f, 0f,
        0f, 0f, 1f
    )

    @Before
    fun setUp() {
        eskf = ESKF()
    }

    @Test
    fun testInitialization() {
        assertEquals(0.0, eskf.position.x, 1e-9)
        assertEquals(0.0, eskf.position.y, 1e-9)
        assertEquals(0.0, eskf.position.z, 1e-9)
        assertEquals(0.0, eskf.velocity.x, 1e-9)
        assertEquals(0.0, eskf.velocity.y, 1e-9)
        assertEquals(0.0, eskf.velocity.z, 1e-9)

        // Covariance diagonal should be positive
        for (i in 0 until 9) {
            assertTrue("P[$i,$i] must be positive", eskf.P[i * 9 + i] > 0.0)
        }
    }

    @Test
    fun testStationaryZuptSuppression() {
        // Phone resting on table: specific force measures +9.81 m/s^2 on Z axis
        val stationaryAccel = floatArrayOf(0f, 0f, 9.81f)
        val dt = 0.005 // 200 Hz (5 ms)

        for (step in 0 until 200) {
            eskf.predict(stationaryAccel, identityR, dt)
            eskf.updateZUPT(0.01)
        }

        // Velocity and horizontal position should remain near zero
        assertEquals(0.0, eskf.velocity.x, 1e-3)
        assertEquals(0.0, eskf.velocity.y, 1e-3)
        assertEquals(0.0, eskf.position.x, 1e-3)
        assertEquals(0.0, eskf.position.y, 1e-3)
    }

    @Test
    fun testVelocityUpdateConvergence() {
        val measuredVel = doubleArrayOf(2.0, -1.0, 0.0)
        eskf.updateVelocity(measuredVel, 0.01)

        // Velocity should pull towards measurement
        assertTrue(eskf.velocity.x > 1.5)
        assertTrue(eskf.velocity.y < -0.7)
    }

    @Test
    fun testCovarianceSymmetry() {
        val accel = floatArrayOf(0.5f, -0.2f, 9.7f)
        eskf.predict(accel, identityR, 0.01)
        eskf.updateVelocity(doubleArrayOf(1.0, 0.0, 0.0), 0.1)
        eskf.updateLateralVelocity(0.0, 0.1)

        // Check symmetry P_ij == P_ji
        for (i in 0 until 9) {
            for (j in 0 until 9) {
                val p_ij = eskf.P[i * 9 + j]
                val p_ji = eskf.P[j * 9 + i]
                assertEquals("P symmetry violated at ($i, $j)", p_ij, p_ji, 1e-9)
            }
        }
    }

    @Test
    fun testLateralVelocityConstraint() {
        // Set initial velocity East = 10, North = 5
        eskf.velocity.x = 10.0
        eskf.velocity.y = 5.0

        // Heading is East (0 rad) -> Lateral axis is North (y). Lateral update should reduce Vy.
        eskf.updateLateralVelocity(0.0, 0.01)

        assertTrue(abs(eskf.velocity.y) < 2.0)
    }

    @Test
    fun testBarometricAltitudeUpdate() {
        // Initial Z is 0.0
        assertEquals(0.0, eskf.position.z, 1e-9)

        // Measured barometric altitude offset = 12.5 meters (e.g. climbed 4 floors)
        eskf.updateAltitude(12.5, 0.1)

        // Position Z should converge toward 12.5
        assertTrue("Position Z should increase toward 12.5m, was ${eskf.position.z}", eskf.position.z > 10.0)
    }
}
