package com.example.inertialgps

import kotlin.math.sqrt

class StepDetector {
    
    // Config
    private val MIN_STEP_DELAY_MS = 350L
    private val MAX_STEP_DELAY_MS = 2000L
    private val PEAK_THRESHOLD = 1.1f // m/s^2 above gravity (middle ground)
    private val VALLEY_THRESHOLD = -0.6f // m/s^2 below gravity
    private val GYRO_VARIANCE_THRESHOLD = 15.0f // reject if gyro variance is too high (swinging)
    
    private val GRAVITY = 9.81f
    private val WEINBERG_K = 0.45f // Tunable constant for step length
    
    // State
    private var lastPeakTime = 0L
    private var lastPeakValue = 0f
    private var currentValleyValue = 0f
    private var isLookingForValley = false
    
    // Gyro buffer for variance calculation
    private val gyroBuffer = FloatArray(50)
    private var gyroIndex = 0
    private var gyroCount = 0

    // PDR Output
    var stepLength = 0f
        private set

    fun process(aWorld: DoubleArray, gx: Float, gy: Float, gz: Float, timestampMs: Long): Boolean {
        // 1. Vertical Acceleration (Z-axis in World Frame)
        // aWorld is already minus gravity, so resting Z is 0.
        val az = aWorld[2].toFloat()
        
        // 2. Horizontal Acceleration (X, Y in World Frame)
        val ax = aWorld[0].toFloat()
        val ay = aWorld[1].toFloat()
        val horizontalMag = sqrt(ax * ax + ay * ay)
        
        // 3. Track Gyroscope Magnitude for variance (swinging bag rejection)
        val gyroMag = sqrt(gx * gx + gy * gy + gz * gz)
        gyroBuffer[gyroIndex] = gyroMag
        gyroIndex = (gyroIndex + 1) % gyroBuffer.size
        if (gyroCount < gyroBuffer.size) gyroCount++

        // 4. Peak/Valley Detection on Vertical Axis
        var stepDetected = false

        if (!isLookingForValley) {
            // Looking for a peak
            if (az > PEAK_THRESHOLD) {
                if (az > lastPeakValue) {
                    lastPeakValue = az
                }
            } else if (lastPeakValue > PEAK_THRESHOLD && az < 0) {
                // Crossed zero after a peak, start looking for valley
                isLookingForValley = true
                currentValleyValue = 0f
            }
        } else {
            // Looking for a valley
            if (az < VALLEY_THRESHOLD) {
                if (az < currentValleyValue) {
                    currentValleyValue = az
                }
            } else if (currentValleyValue < VALLEY_THRESHOLD && az > 0) {
                // Crossed zero after a valley, step complete
                
                val dt = timestampMs - lastPeakTime
                if (dt in MIN_STEP_DELAY_MS..MAX_STEP_DELAY_MS) {
                    // Check Gyro Variance and Horizontal Movement to ensure it's a real step
                    val gyroVar = calculateGyroVariance()
                    if (gyroVar < GYRO_VARIANCE_THRESHOLD && horizontalMag > 0.3f) {
                        // VALID STEP!
                        stepLength = WEINBERG_K * sqrt(sqrt(lastPeakValue - currentValleyValue))
                        // Constrain step length to realistic values (0.4m to 1.2m)
                        if (stepLength < 0.4f) stepLength = 0.4f
                        if (stepLength > 1.2f) stepLength = 1.2f
                        
                        stepDetected = true
                        lastPeakTime = timestampMs
                    }
                } else if (dt > MAX_STEP_DELAY_MS) {
                    // Timeout or first step
                    lastPeakTime = timestampMs
                }
                
                // Reset for next step
                isLookingForValley = false
                lastPeakValue = 0f
                currentValleyValue = 0f
            }
        }
        return stepDetected
    }

    private fun calculateGyroVariance(): Float {
        if (gyroCount == 0) return 0f
        var sum = 0f
        for (i in 0 until gyroCount) sum += gyroBuffer[i]
        val mean = sum / gyroCount
        var variance = 0f
        for (i in 0 until gyroCount) {
            val diff = gyroBuffer[i] - mean
            variance += diff * diff
        }
        return variance / gyroCount
    }
}
