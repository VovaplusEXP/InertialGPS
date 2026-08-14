package com.example.inertialgps

import kotlin.math.*

/**
 * Lightweight 3D Vector for zero-allocation state vector representation.
 * Supports legacy indexing `.get(row, col)` and `.fill(value)` for compatibility.
 */
class Vector3(val data: DoubleArray = DoubleArray(3)) {
    inline var x: Double
        get() = data[0]
        set(value) { data[0] = value }

    inline var y: Double
        get() = data[1]
        set(value) { data[1] = value }

    inline var z: Double
        get() = data[2]
        set(value) { data[2] = value }

    fun fill(value: Double) {
        data[0] = value
        data[1] = value
        data[2] = value
    }

    fun get(row: Int, col: Int = 0): Double = data[row]

    fun set(row: Int, col: Int = 0, value: Double) {
        data[row] = value
    }
}

/**
 * High-Performance Zero-Allocation Error-State Kalman Filter (ESKF) for Android SINS/PDR.
 * Tracks 9 nominal states: Position (3), Velocity (3), Accelerometer Bias (3).
 * Attitude is provided by Android's hardware Rotation Vector.
 */
class ESKF {
    // Nominal states (3D vectors)
    val position = Vector3()
    val velocity = Vector3()
    val accelBias = Vector3()

    // Covariance matrix 9x9 stored in row-major order (81 elements)
    val P = DoubleArray(81)

    // Process noise diagonal (9 elements)
    private val Q = DoubleArray(9)

    // Reusable preallocated buffers to guarantee 0 heap allocations in 200 Hz loop
    private val tempM = DoubleArray(81)
    private val invS = DoubleArray(9)
    private val K = DoubleArray(27) // 9x3 Kalman Gain
    private val dx = DoubleArray(9)
    private val PHT = DoubleArray(9)
    private val zeroVel = DoubleArray(3)

    val linearAccelWorld = DoubleArray(3)

    init {
        reset()
        // Tune Q (Process Noise)
        Q[0] = 0.0001; Q[1] = 0.0001; Q[2] = 0.0001 // Position
        Q[3] = 0.001;  Q[4] = 0.001;  Q[5] = 0.001  // Velocity
        Q[6] = 1e-5;   Q[7] = 1e-5;   Q[8] = 1e-5   // Accel bias random walk
    }

    fun reset() {
        position.fill(0.0)
        velocity.fill(0.0)
        accelBias.fill(0.0)
        linearAccelWorld.fill(0.0)
        P.fill(0.0)
        for (i in 0 until 9) {
            P[i * 9 + i] = 0.01
        }
    }

    /**
     * Propagates the nominal state and error covariance over dt seconds.
     * Guaranteed zero-allocation.
     */
    fun predict(accelRaw: FloatArray, R_array: FloatArray, dt: Double) {
        if (dt <= 0.0) return

        // 1. Specific force in body frame minus estimated bias
        val ab0 = accelRaw[0].toDouble() - accelBias.x
        val ab1 = accelRaw[1].toDouble() - accelBias.y
        val ab2 = accelRaw[2].toDouble() - accelBias.z

        // R_array from Android is 3x3 in row-major order
        val r00 = R_array[0].toDouble(); val r01 = R_array[1].toDouble(); val r02 = R_array[2].toDouble()
        val r10 = R_array[3].toDouble(); val r11 = R_array[4].toDouble(); val r12 = R_array[5].toDouble()
        val r20 = R_array[6].toDouble(); val r21 = R_array[7].toDouble(); val r22 = R_array[8].toDouble()

        // Rotate acceleration to world frame and subtract gravity (9.81 m/s^2 along Z)
        val aw0 = r00 * ab0 + r01 * ab1 + r02 * ab2
        val aw1 = r10 * ab0 + r11 * ab1 + r12 * ab2
        val aw2 = r20 * ab0 + r21 * ab1 + r22 * ab2 - 9.81

        linearAccelWorld[0] = aw0
        linearAccelWorld[1] = aw1
        linearAccelWorld[2] = aw2

        // Nominal state integration
        val dt2_2 = 0.5 * dt * dt

        position.x += velocity.x * dt + aw0 * dt2_2
        position.y += velocity.y * dt + aw1 * dt2_2
        position.z += velocity.z * dt + aw2 * dt2_2

        velocity.x += aw0 * dt
        velocity.y += aw1 * dt
        velocity.z += aw2 * dt

        // 2. Propagate Covariance: P = F * P * F^T + Q
        // Jacobian block entries:
        // F_pa = -0.5 * R * dt^2
        val fpa00 = -r00 * dt2_2; val fpa01 = -r01 * dt2_2; val fpa02 = -r02 * dt2_2
        val fpa10 = -r10 * dt2_2; val fpa11 = -r11 * dt2_2; val fpa12 = -r12 * dt2_2
        val fpa20 = -r20 * dt2_2; val fpa21 = -r21 * dt2_2; val fpa22 = -r22 * dt2_2

        // F_va = -R * dt
        val fva00 = -r00 * dt; val fva01 = -r01 * dt; val fva02 = -r02 * dt
        val fva10 = -r10 * dt; val fva11 = -r11 * dt; val fva12 = -r12 * dt
        val fva20 = -r20 * dt; val fva21 = -r21 * dt; val fva22 = -r22 * dt

        // Compute tempM = F * P (9x9)
        for (c in 0 until 9) {
            val p0c = P[c];      val p1c = P[9 + c];  val p2c = P[18 + c]
            val p3c = P[27 + c]; val p4c = P[36 + c]; val p5c = P[45 + c]
            val p6c = P[54 + c]; val p7c = P[63 + c]; val p8c = P[72 + c]

            // Row 0..2: Pos
            tempM[c]      = p0c + dt * p3c + (fpa00 * p6c + fpa01 * p7c + fpa02 * p8c)
            tempM[9 + c]  = p1c + dt * p4c + (fpa10 * p6c + fpa11 * p7c + fpa12 * p8c)
            tempM[18 + c] = p2c + dt * p5c + (fpa20 * p6c + fpa21 * p7c + fpa22 * p8c)

            // Row 3..5: Vel
            tempM[27 + c] = p3c + (fva00 * p6c + fva01 * p7c + fva02 * p8c)
            tempM[36 + c] = p4c + (fva10 * p6c + fva11 * p7c + fva12 * p8c)
            tempM[45 + c] = p5c + (fva20 * p6c + fva21 * p7c + fva22 * p8c)

            // Row 6..8: Accel Bias
            tempM[54 + c] = p6c
            tempM[63 + c] = p7c
            tempM[72 + c] = p8c
        }

        // Compute P = tempM * F^T + Q
        for (r in 0 until 9) {
            val rOffset = r * 9
            val m_r0 = tempM[rOffset];     val m_r1 = tempM[rOffset + 1]; val m_r2 = tempM[rOffset + 2]
            val m_r3 = tempM[rOffset + 3]; val m_r4 = tempM[rOffset + 4]; val m_r5 = tempM[rOffset + 5]
            val m_r6 = tempM[rOffset + 6]; val m_r7 = tempM[rOffset + 7]; val m_r8 = tempM[rOffset + 8]

            P[rOffset]     = m_r0 + dt * m_r3 + (m_r6 * fpa00 + m_r7 * fpa01 + m_r8 * fpa02)
            P[rOffset + 1] = m_r1 + dt * m_r4 + (m_r6 * fpa10 + m_r7 * fpa11 + m_r8 * fpa12)
            P[rOffset + 2] = m_r2 + dt * m_r5 + (m_r6 * fpa20 + m_r7 * fpa21 + m_r8 * fpa22)

            P[rOffset + 3] = m_r3 + (m_r6 * fva00 + m_r7 * fva01 + m_r8 * fva02)
            P[rOffset + 4] = m_r4 + (m_r6 * fva10 + m_r7 * fva11 + m_r8 * fva12)
            P[rOffset + 5] = m_r5 + (m_r6 * fva20 + m_r7 * fva21 + m_r8 * fva22)

            P[rOffset + 6] = m_r6
            P[rOffset + 7] = m_r7
            P[rOffset + 8] = m_r8

            P[rOffset + r] += Q[r]
        }

        symmetrizeP()
    }

    /**
     * Updates the ESKF with a 3D position measurement (H observes states 0..2).
     */
    fun updatePosition(measuredPos: DoubleArray, R_cov: Double) {
        val z0 = measuredPos[0] - position.x
        val z1 = measuredPos[1] - position.y
        val z2 = measuredPos[2] - position.z

        // S = P[0..2, 0..2] + R_cov * I_3
        val s00 = P[0] + R_cov; val s01 = P[1];         val s02 = P[2]
        val s10 = P[9];         val s11 = P[10] + R_cov; val s12 = P[11]
        val s20 = P[18];        val s21 = P[19];        val s22 = P[20] + R_cov

        if (!invert3x3(s00, s01, s02, s10, s11, s12, s20, s21, s22)) return

        // Kalman Gain K = P[:, 0..2] * invS (9x3)
        for (i in 0 until 9) {
            val p_i0 = P[i * 9]; val p_i1 = P[i * 9 + 1]; val p_i2 = P[i * 9 + 2]
            K[i * 3]     = p_i0 * invS[0] + p_i1 * invS[3] + p_i2 * invS[6]
            K[i * 3 + 1] = p_i0 * invS[1] + p_i1 * invS[4] + p_i2 * invS[7]
            K[i * 3 + 2] = p_i0 * invS[2] + p_i1 * invS[5] + p_i2 * invS[8]
        }

        // dx = K * z
        for (i in 0 until 9) {
            dx[i] = K[i * 3] * z0 + K[i * 3 + 1] * z1 + K[i * 3 + 2] * z2
        }

        injectErrorState()

        // P = P - K * P[0..2, :]
        for (i in 0 until 9) {
            val k0 = K[i * 3]; val k1 = K[i * 3 + 1]; val k2 = K[i * 3 + 2]
            val iOffset = i * 9
            for (j in 0 until 9) {
                P[iOffset + j] -= (k0 * P[j] + k1 * P[9 + j] + k2 * P[18 + j])
            }
        }

        symmetrizeP()
    }

    /**
     * Updates the ESKF with a 3D velocity measurement (H observes states 3..5).
     */
    fun updateVelocity(measuredVel: DoubleArray, R_cov: Double) {
        val z0 = measuredVel[0] - velocity.x
        val z1 = measuredVel[1] - velocity.y
        val z2 = measuredVel[2] - velocity.z

        // S = P[3..5, 3..5] + R_cov * I_3
        val s00 = P[30] + R_cov; val s01 = P[31];         val s02 = P[32]
        val s10 = P[39];         val s11 = P[40] + R_cov; val s12 = P[41]
        val s20 = P[48];         val s21 = P[49];         val s22 = P[50] + R_cov

        if (!invert3x3(s00, s01, s02, s10, s11, s12, s20, s21, s22)) return

        // Kalman Gain K = P[:, 3..5] * invS (9x3)
        for (i in 0 until 9) {
            val p_i3 = P[i * 9 + 3]; val p_i4 = P[i * 9 + 4]; val p_i5 = P[i * 9 + 5]
            K[i * 3]     = p_i3 * invS[0] + p_i4 * invS[3] + p_i5 * invS[6]
            K[i * 3 + 1] = p_i3 * invS[1] + p_i4 * invS[4] + p_i5 * invS[7]
            K[i * 3 + 2] = p_i3 * invS[2] + p_i4 * invS[5] + p_i5 * invS[8]
        }

        // dx = K * z
        for (i in 0 until 9) {
            dx[i] = K[i * 3] * z0 + K[i * 3 + 1] * z1 + K[i * 3 + 2] * z2
        }

        injectErrorState()

        // P = P - K * P[3..5, :]
        for (i in 0 until 9) {
            val k0 = K[i * 3]; val k1 = K[i * 3 + 1]; val k2 = K[i * 3 + 2]
            val iOffset = i * 9
            for (j in 0 until 9) {
                P[iOffset + j] -= (k0 * P[27 + j] + k1 * P[36 + j] + k2 * P[45 + j])
            }
        }

        symmetrizeP()
    }

    /**
     * Non-Holonomic Constraint (NHC): Lateral velocity update (1D scalar observation).
     */
    fun updateLateralVelocity(heading: Double, R_cov: Double) {
        val sinH = sin(heading)
        val cosH = cos(heading)

        val currentLatVel = -sinH * velocity.x + cosH * velocity.y
        val z = -currentLatVel

        // PHT = P * H^T (9x1 column vector)
        for (i in 0 until 9) {
            PHT[i] = -sinH * P[i * 9 + 3] + cosH * P[i * 9 + 4]
        }

        // S = H * P * H^T + R_cov (scalar)
        val S = (-sinH * PHT[3] + cosH * PHT[4]) + R_cov
        if (S <= 1e-12) return

        val invScalarS = 1.0 / S

        // Kalman Gain K = PHT / S and dx = K * z
        for (i in 0 until 9) {
            val ki = PHT[i] * invScalarS
            dx[i] = ki * z
            // Update covariance P = P - K * HP where HP_j = PHT_j
            val iOffset = i * 9
            for (j in 0 until 9) {
                P[iOffset + j] -= ki * PHT[j]
            }
        }

        injectErrorState()
        symmetrizeP()
    }

    /**
     * Altitude/Barometer Update (1D scalar observation on Z-position, state index 2).
     */
    fun updateAltitude(measuredZ: Double, R_cov: Double) {
        val z = measuredZ - position.z

        // S = P[2, 2] + R_cov (scalar)
        val S = P[20] + R_cov
        if (S <= 1e-12) return

        val invS = 1.0 / S

        // Kalman Gain K = P[:, 2] / S and dx = K * z
        for (i in 0 until 9) {
            val ki = P[i * 9 + 2] * invS
            dx[i] = ki * z
            // Update covariance P = P - K * P[2, :] where row 2 of P is P[18..26]
            val iOffset = i * 9
            for (j in 0 until 9) {
                P[iOffset + j] -= ki * P[18 + j]
            }
        }

        injectErrorState()
        symmetrizeP()
    }

    /**
     * Zero Velocity Update (ZUPT).
     */
    fun updateZUPT(R_cov: Double) {
        updateVelocity(zeroVel, R_cov)
    }

    private fun injectErrorState() {
        position.x += dx[0]; position.y += dx[1]; position.z += dx[2]
        velocity.x += dx[3]; velocity.y += dx[4]; velocity.z += dx[5]
        accelBias.x += dx[6]; accelBias.y += dx[7]; accelBias.z += dx[8]
    }

    private fun symmetrizeP() {
        for (i in 0 until 9) {
            for (j in i + 1 until 9) {
                val sym = 0.5 * (P[i * 9 + j] + P[j * 9 + i])
                P[i * 9 + j] = sym
                P[j * 9 + i] = sym
            }
        }
    }

    private fun invert3x3(
        a00: Double, a01: Double, a02: Double,
        a10: Double, a11: Double, a12: Double,
        a20: Double, a21: Double, a22: Double
    ): Boolean {
        val c00 = a11 * a22 - a12 * a21
        val c01 = a12 * a20 - a10 * a22
        val c02 = a10 * a21 - a11 * a20

        val det = a00 * c00 + a01 * c01 + a02 * c02
        if (abs(det) < 1e-12) return false

        val invDet = 1.0 / det

        val c10 = a02 * a21 - a01 * a22
        val c11 = a00 * a22 - a02 * a20
        val c12 = a01 * a20 - a00 * a21

        val c20 = a01 * a12 - a02 * a11
        val c21 = a02 * a10 - a00 * a12
        val c22 = a00 * a11 - a01 * a10

        // Transpose cofactor matrix for adjugate and multiply by 1/det
        invS[0] = c00 * invDet; invS[1] = c10 * invDet; invS[2] = c20 * invDet
        invS[3] = c01 * invDet; invS[4] = c11 * invDet; invS[5] = c21 * invDet
        invS[6] = c02 * invDet; invS[7] = c12 * invDet; invS[8] = c22 * invDet

        return true
    }
}
