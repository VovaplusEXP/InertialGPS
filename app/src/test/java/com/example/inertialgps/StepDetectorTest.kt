package com.example.inertialgps

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import kotlin.math.sin

class StepDetectorTest {

    private lateinit var stepDetector: StepDetector

    @Before
    fun setUp() {
        stepDetector = StepDetector()
    }

    @Test
    fun testIndoorWalkingStepDetection() {
        var stepsDetected = 0
        var timestamp = 1000000L
        val dtMs = 5L // 200 Hz

        // Simulate 10 steps of indoor walking at 1.8 Hz cadence (step interval ~555ms)
        val stepFreq = 1.8 // Hz
        val stepPeriodSec = 1.0 / stepFreq
        val totalTimeSec = 6.0
        val totalSamples = (totalTimeSec / 0.005).toInt()

        for (i in 0 until totalSamples) {
            val tSec = i * 0.005
            timestamp += dtMs

            // Sinusoidal acceleration wave + small vertical impact spike (typical indoor walking)
            val azWorld = (0.7 * sin(2.0 * Math.PI * stepFreq * tSec)).toDouble()
            val aWorld = doubleArrayOf(0.1, 0.1, azWorld)

            val isStep = stepDetector.process(
                aWorld = aWorld,
                gx = 0.05f,
                gy = 0.02f,
                gz = 0.01f,
                vx = 0.0f,
                vy = 0.0f,
                timestampMs = timestamp
            )

            if (isStep) {
                stepsDetected++
                assertTrue("Step length must be realistic, was ${stepDetector.stepLength}", stepDetector.stepLength in 0.4f..1.1f)
                assertTrue("Step velocity must be realistic, was ${stepDetector.stepVelocity}", stepDetector.stepVelocity in 0.5f..2.2f)
            }
        }

        // Over 6 seconds at 1.8 Hz, expected ~10-11 steps
        assertTrue("Expected between 8 and 12 steps, detected $stepsDetected", stepsDetected in 8..12)
    }

    @Test
    fun testStationaryNoFalseSteps() {
        var stepsDetected = 0
        var timestamp = 1000000L
        val dtMs = 5L

        // 5 seconds of stationary noise (tiny jitter < 0.05 m/s^2)
        for (i in 0 until 1000) {
            timestamp += dtMs
            val noise = (Math.sin(i.toDouble()) * 0.03)
            val aWorld = doubleArrayOf(0.0, 0.0, noise)

            val isStep = stepDetector.process(
                aWorld = aWorld,
                gx = 0.0f,
                gy = 0.0f,
                gz = 0.0f,
                vx = 0.0f,
                vy = 0.0f,
                timestampMs = timestamp
            )

            if (isStep) {
                stepsDetected++
            }
        }

        assertEquals("Stationary state should produce 0 steps", 0, stepsDetected)
    }
}
