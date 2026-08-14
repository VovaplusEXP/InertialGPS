package com.example.inertialgps

import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Adaptive Pedestrian Dead Reckoning (PDR) Step Detector.
 *
 * Uses continuous sliding-window statistics (Moving Average & Moving Variance)
 * to dynamically adjust peak/valley thresholds for both delicate indoor walking
 * and vigorous outdoor walking.
 *
 * Step length is estimated using a hybrid Weinberg + Cadence model:
 *   L = K * [ 0.4 * (a_max - a_min)^(1/4) + 0.25 * freq ]
 */
class StepDetector {

    // Human biomechanical step timing constraints
    private val MIN_STEP_INTERVAL_MS = 240L  // Max cadence ~4.1 Hz (running/fast walk)
    private val MAX_STEP_INTERVAL_MS = 1400L // Min cadence ~0.7 Hz (slow indoor stroll)

    // User / GNSS-calibrated stride multiplier
    var stepK_multiplier = 1.0f

    // Output Step Metrics
    var stepLength = 0f
        private set
    var stepVelocity = 0f
        private set

    // Sliding window for acceleration magnitude statistics (50 samples ~ 250ms at 200Hz)
    private val windowSize = 50
    private val accelWindow = FloatArray(windowSize)
    private var windowIndex = 0
    private var windowCount = 0

    // Low-pass filter for smooth wave extraction (removes high-frequency jitter)
    private var filteredAccel = 0f
    private val lpfAlpha = 0.2f // Cutoff ~3.5 Hz at 200Hz

    // State machine for adaptive peak-valley detection
    private var state = State.SEARCH_PEAK
    private var localMax = Float.NEGATIVE_INFINITY
    private var localMin = Float.POSITIVE_INFINITY
    private var lastPeakTime = 0L
    private var lastStepTime = 0L

    // Step cadence tracking for auto-calibration stability
    private val intervalHistory = LongArray(6)
    private var intervalIndex = 0
    private var intervalCount = 0

    val isCadenceStable: Boolean
        get() {
            if (intervalCount < 4) return false
            var sum = 0.0
            for (i in 0 until intervalCount) {
                sum += intervalHistory[i]
            }
            val mean = sum / intervalCount
            var variance = 0.0
            for (i in 0 until intervalCount) {
                val diff = intervalHistory[i] - mean
                variance += diff * diff
            }
            val stdDevMs = sqrt(variance / intervalCount)
            return stdDevMs < 120.0 // Cadence jitter < 120ms
        }

    private enum class State {
        SEARCH_PEAK,
        SEARCH_VALLEY
    }

    /**
     * Processes inertial sensor sample.
     *
     * @param aWorld Linear acceleration in Earth/World frame [ax, ay, az] (gravity removed)
     * @param gx Gyroscope X (rad/s)
     * @param gy Gyroscope Y (rad/s)
     * @param gz Gyroscope Z (rad/s)
     * @param vx Current ESKF velocity X (m/s)
     * @param vy Current ESKF velocity Y (m/s)
     * @param timestampMs Current epoch timestamp in milliseconds
     * @return true if a valid pedestrian step is detected at this epoch
     */
    fun process(
        aWorld: DoubleArray,
        gx: Float,
        gy: Float,
        gz: Float,
        vx: Float,
        vy: Float,
        timestampMs: Long
    ): Boolean {
        // Vertical world acceleration + dynamic norm combination (immune to phone orientation)
        val azWorld = aWorld[2].toFloat()
        val horizMag = sqrt(aWorld[0] * aWorld[0] + aWorld[1] * aWorld[1]).toFloat()
        val totalNorm = sqrt(azWorld * azWorld + horizMag * horizMag)

        // Low-pass filtered signal
        filteredAccel = filteredAccel + lpfAlpha * (totalNorm - filteredAccel)

        // Update sliding window
        accelWindow[windowIndex] = filteredAccel
        windowIndex = (windowIndex + 1) % windowSize
        if (windowCount < windowSize) windowCount++

        // Calculate moving average and standard deviation
        var sum = 0f
        for (i in 0 until windowCount) {
            sum += accelWindow[i]
        }
        val mean = sum / windowCount

        var varSum = 0f
        for (i in 0 until windowCount) {
            val d = accelWindow[i] - mean
            varSum += d * d
        }
        val stdDev = sqrt(varSum / windowCount)

        // Adaptive thresholds
        // Quiet indoor texting: stdDev ~ 0.15 -> peak delta ~ 0.15 m/s^2
        // Dynamic walking: stdDev ~ 0.8 -> peak delta ~ 0.5 m/s^2
        val dynamicPeakDelta = (0.5f * stdDev).coerceIn(0.12f, 0.9f)
        val peakThreshold = mean + dynamicPeakDelta
        val valleyThreshold = mean - dynamicPeakDelta

        var isStepDetected = false

        when (state) {
            State.SEARCH_PEAK -> {
                if (filteredAccel > localMax) {
                    localMax = filteredAccel
                }
                // Peak confirmed when signal starts descending below threshold
                if (localMax > peakThreshold && filteredAccel < (localMax - 0.05f)) {
                    state = State.SEARCH_VALLEY
                    localMin = filteredAccel
                    lastPeakTime = timestampMs
                }
            }

            State.SEARCH_VALLEY -> {
                if (filteredAccel < localMin) {
                    localMin = filteredAccel
                }
                // Valley confirmed when signal rebounds back up
                if (localMin < valleyThreshold && filteredAccel > (localMin + 0.05f)) {
                    val dt = if (lastStepTime > 0L) (timestampMs - lastStepTime) else 550L

                    // Check physiological step interval constraints
                    if (dt in MIN_STEP_INTERVAL_MS..MAX_STEP_INTERVAL_MS) {
                        val peakValleyDiff = max(localMax - localMin, 0.1f)
                        val freq = (1000.0f / dt.toFloat()).coerceIn(0.7f, 4.0f)

                        // Hybrid Weinberg + Cadence step length formula
                        val weinberg = peakValleyDiff.toDouble().pow(0.25).toFloat()
                        val rawLength = (0.42f * weinberg + 0.22f * freq) * stepK_multiplier
                        stepLength = rawLength.coerceIn(0.35f, 1.25f)

                        // Step velocity = Step Length / Step Duration
                        stepVelocity = (stepLength / (dt / 1000.0f)).coerceIn(0.4f, 2.5f)

                        // Record interval for cadence stability assessment
                        intervalHistory[intervalIndex] = dt
                        intervalIndex = (intervalIndex + 1) % intervalHistory.size
                        if (intervalCount < intervalHistory.size) intervalCount++

                        isStepDetected = true
                        lastStepTime = timestampMs
                    } else if (dt > MAX_STEP_INTERVAL_MS) {
                        // Reset timestamp if gap was too large
                        lastStepTime = timestampMs
                    }

                    // Reset for next peak
                    state = State.SEARCH_PEAK
                    localMax = filteredAccel
                    localMin = Float.POSITIVE_INFINITY
                }
            }
        }

        return isStepDetected
    }
}
