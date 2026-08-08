package com.example.inertialgps

import kotlin.math.sqrt

class StepDetector {
    
    // Config
    private val MIN_STEP_DELAY_MS = 350L
    private val MAX_STEP_DELAY_MS = 2000L
    private val GYRO_VARIANCE_THRESHOLD = 15.0f
    
    private val WEINBERG_K = 0.45f // Tunable constant for step length
    
    // Dynamic Thresholds
    private var dynamicPeakThreshold = 1.0f
    private var dynamicValleyThreshold = -0.5f
    
    // State
    private var lastPeakTime = 0L
    private var lastValidStepTime = 0L
    private var lastPeakValue = 0f
    private var currentValleyValue = 0f
    private var isLookingForValley = false
    
    // Buffers for variance calculation
    private val gyroBuffer = FloatArray(50)
    private var gyroIndex = 0
    private var gyroCount = 0
    
    // High-Pass Filter state for Z-axis
    private var azDC = 0f
    private var azPrev = 0f
    
    // 200 samples = ~1 sec at 200Hz
    private val zBuffer = FloatArray(200)
    private var zIndex = 0
    private var zCount = 0

    // PDR Output
    var stepLength = 0f
        private set

    fun process(aWorld: DoubleArray, gx: Float, gy: Float, gz: Float, vx: Float, vy: Float, timestampMs: Long): Boolean {
        val azRaw = aWorld[2].toFloat()
        
        // High-pass filter (DC removal) to eliminate false steps caused by rotating the phone's hardware bias
        azDC = azDC + 0.005f * (azRaw - azDC)
        val az = azRaw - azDC
        val ax = aWorld[0].toFloat()
        val ay = aWorld[1].toFloat()
        val horizontalMag = sqrt(ax * ax + ay * ay)
        
        // Track Gyroscope
        val gyroMag = sqrt(gx * gx + gy * gy + gz * gz)
        gyroBuffer[gyroIndex] = gyroMag
        gyroIndex = (gyroIndex + 1) % gyroBuffer.size
        if (gyroCount < gyroBuffer.size) gyroCount++
        
        // Track Z-axis for dynamic thresholding
        zBuffer[zIndex] = az
        zIndex = (zIndex + 1) % zBuffer.size
        if (zCount < zBuffer.size) zCount++
        
        updateDynamicThresholds()

        var stepDetected = false

        if (!isLookingForValley) {
            // Looking for a peak
            if (az > dynamicPeakThreshold) {
                if (az > lastPeakValue) {
                    lastPeakValue = az
                }
            } else if (lastPeakValue > dynamicPeakThreshold && az < 0) {
                isLookingForValley = true
                currentValleyValue = 0f
            }
        } else {
            // Looking for a valley
            if (az < dynamicValleyThreshold) {
                if (az < currentValleyValue) {
                    currentValleyValue = az
                }
            } else if (currentValleyValue < dynamicValleyThreshold && az > 0) {
                val dt = timestampMs - lastPeakTime
                if (dt in MIN_STEP_DELAY_MS..MAX_STEP_DELAY_MS) {
                    val gyroVar = calculateVariance(gyroBuffer, gyroCount)
                    val velMag = sqrt(vx * vx + vy * vy)
                    
                    // 1. Strict Step Throttling (250ms absolute minimum between accepted steps)
                    // 2. Gyro variance check (reject wild swinging)
                    // 3. True Physical Velocity check (ESKF must see actual movement > 0.15 m/s)
                    if (timestampMs - lastValidStepTime > 250L && gyroVar < GYRO_VARIANCE_THRESHOLD && velMag > 0.15f) {
                        stepLength = WEINBERG_K * sqrt(sqrt(lastPeakValue - currentValleyValue))
                        if (stepLength < 0.4f) stepLength = 0.4f
                        if (stepLength > 1.2f) stepLength = 1.2f
                        
                        stepDetected = true
                        lastValidStepTime = timestampMs
                    }
                    // Always update lastPeakTime to prevent overlapping fast noise from triggering later
                    lastPeakTime = timestampMs
                } else if (dt > MAX_STEP_DELAY_MS) {
                    lastPeakTime = timestampMs
                } else {
                    // dt < MIN_STEP_DELAY_MS, update peak time anyway to reject noise clumps
                    lastPeakTime = timestampMs
                }
                
                isLookingForValley = false
                lastPeakValue = 0f
                currentValleyValue = 0f
            }
        }
        return stepDetected
    }

    private fun updateDynamicThresholds() {
        if (zCount < 50) return
        val zVar = calculateVariance(zBuffer, zCount)
        
        // Texting Mode (low variance): threshold drops to ~0.45
        // Pocket Mode (high variance): threshold rises to ~1.5
        dynamicPeakThreshold = 0.4f + 0.3f * zVar
        if (dynamicPeakThreshold > 1.5f) dynamicPeakThreshold = 1.5f
        if (dynamicPeakThreshold < 0.45f) dynamicPeakThreshold = 0.45f
        
        dynamicValleyThreshold = -(0.25f + 0.2f * zVar)
        if (dynamicValleyThreshold < -1.0f) dynamicValleyThreshold = -1.0f
        if (dynamicValleyThreshold > -0.3f) dynamicValleyThreshold = -0.3f
    }

    private fun calculateVariance(buffer: FloatArray, count: Int): Float {
        if (count == 0) return 0f
        var sum = 0f
        for (i in 0 until count) sum += buffer[i]
        val mean = sum / count
        var variance = 0f
        for (i in 0 until count) {
            val diff = buffer[i] - mean
            variance += diff * diff
        }
        return variance / count
    }
}
