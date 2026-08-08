package com.example.inertialgps

import org.ejml.simple.SimpleMatrix
import kotlin.math.*

class ESKF {
    // Nominal states
    var position = SimpleMatrix(3, 1)
    var velocity = SimpleMatrix(3, 1)
    var accelBias = SimpleMatrix(3, 1)
    
    // Covariance matrix 9x9 (Position, Velocity, AccelBias)
    var P = SimpleMatrix.identity(9).scale(0.01)
    
    // Process noise covariance 9x9
    private val Q = SimpleMatrix.identity(9)
    
    private val gravity = SimpleMatrix(3, 1).apply { set(2, 0, 9.81) }
    
    init {
        // Tune Q (Process Noise)
        for (i in 0..2) Q.set(i, i, 0.0001) // Pos
        for (i in 3..5) Q.set(i, i, 0.001)  // Vel
        for (i in 6..8) Q.set(i, i, 1e-5)   // Accel bias random walk
    }

    var linearAccelWorld = doubleArrayOf(0.0, 0.0, 0.0)
        private set
        
    fun predict(accelRaw: FloatArray, R_array: FloatArray, dt: Double) {
        if (dt <= 0) return
        
        val aRaw = SimpleMatrix(3, 1).apply { 
            set(0, 0, accelRaw[0].toDouble())
            set(1, 0, accelRaw[1].toDouble())
            set(2, 0, accelRaw[2].toDouble())
        }
        
        val a = aRaw.minus(accelBias)
        
        // Build Rotation Matrix from Android's hardware array
        val R = SimpleMatrix(3, 3)
        R.set(0, 0, R_array[0].toDouble()); R.set(0, 1, R_array[1].toDouble()); R.set(0, 2, R_array[2].toDouble())
        R.set(1, 0, R_array[3].toDouble()); R.set(1, 1, R_array[4].toDouble()); R.set(1, 2, R_array[5].toDouble())
        R.set(2, 0, R_array[6].toDouble()); R.set(2, 1, R_array[7].toDouble()); R.set(2, 2, R_array[8].toDouble())
        
        // 1. Update nominal state
        val aWorld = R.mult(a).minus(gravity)
        linearAccelWorld[0] = aWorld.get(0,0)
        linearAccelWorld[1] = aWorld.get(1,0)
        linearAccelWorld[2] = aWorld.get(2,0)
        
        position = position.plus(velocity.scale(dt)).plus(aWorld.scale(0.5 * dt * dt))
        velocity = velocity.plus(aWorld.scale(dt))
        
        // 2. Propagate Covariance
        val F = computeJacobian(R, dt)
        P = F.mult(P).mult(F.transpose()).plus(Q)
    }
    
    // Updates the ESKF with a position measurement (e.g., from PDR)
    fun updatePosition(measuredPos: DoubleArray, R_cov: Double) {
        val H = SimpleMatrix(3, 9)
        H.set(0, 0, 1.0); H.set(1, 1, 1.0); H.set(2, 2, 1.0)
        
        val R_mat = SimpleMatrix.identity(3).scale(R_cov)
        
        val S = H.mult(P).mult(H.transpose()).plus(R_mat)
        val K = P.mult(H.transpose()).mult(S.invert())
        
        val z = SimpleMatrix(3, 1).apply {
            set(0, 0, measuredPos[0])
            set(1, 0, measuredPos[1])
            set(2, 0, measuredPos[2])
        }.minus(position)
        
        val dx = K.mult(z)
        injectErrorState(dx)
        
        val I = SimpleMatrix.identity(9)
        P = I.minus(K.mult(H)).mult(P)
    }
    
    // Zero Velocity Update
    fun updateZUPT(R_cov: Double) {
        val H = SimpleMatrix(3, 9)
        H.set(0, 3, 1.0); H.set(1, 4, 1.0); H.set(2, 5, 1.0)
        
        val R_mat = SimpleMatrix.identity(3).scale(R_cov)
        
        val S = H.mult(P).mult(H.transpose()).plus(R_mat)
        val K = P.mult(H.transpose()).mult(S.invert())
        
        val z = SimpleMatrix(3, 1).minus(velocity) // 0 - nominal velocity
        
        val dx = K.mult(z)
        injectErrorState(dx)
        
        val I = SimpleMatrix.identity(9)
        P = I.minus(K.mult(H)).mult(P)
    }

    private fun injectErrorState(dx: SimpleMatrix) {
        // dx: dp(0..2), dv(3..5), dba(6..8)
        position = position.plus(dx.extractMatrix(0, 3, 0, 1))
        velocity = velocity.plus(dx.extractMatrix(3, 6, 0, 1))
        accelBias = accelBias.plus(dx.extractMatrix(6, 9, 0, 1))
    }

    private fun computeJacobian(R: SimpleMatrix, dt: Double): SimpleMatrix {
        val F = SimpleMatrix.identity(9)
        
        // dp / dv = I * dt
        F.set(0, 3, dt); F.set(1, 4, dt); F.set(2, 5, dt)
        
        // dp / dba = -0.5 * R * dt^2
        val dp_dba = R.scale(-0.5 * dt * dt)
        F.insertIntoThis(0, 6, dp_dba)
        
        // dv / dba = -R * dt
        val dv_dba = R.scale(-dt)
        F.insertIntoThis(3, 6, dv_dba)
        
        return F
    }
}
