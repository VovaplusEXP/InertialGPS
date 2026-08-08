package com.example.inertialgps

import org.ejml.simple.SimpleMatrix
import kotlin.math.*

class ESKF {
    // Nominal states
    var position = SimpleMatrix(3, 1)
    var velocity = SimpleMatrix(3, 1)
    var quaternion = SimpleMatrix(4, 1).apply { set(0, 0, 1.0) } // [w, x, y, z]
    var accelBias = SimpleMatrix(3, 1)
    var gyroBias = SimpleMatrix(3, 1)
    
    // Covariance matrix 15x15
    var P = SimpleMatrix.identity(15).scale(0.01)
    
    // Process noise covariance 15x15
    private val Q = SimpleMatrix.identity(15)
    
    private val gravity = SimpleMatrix(3, 1).apply { set(2, 0, 9.81) }
    
    init {
        // Tune Q (Process Noise)
        for (i in 0..2) Q.set(i, i, 0.0001) // Pos
        for (i in 3..5) Q.set(i, i, 0.001)  // Vel
        for (i in 6..8) Q.set(i, i, 0.0001)  // Ori
        for (i in 9..11) Q.set(i, i, 1e-5) // Accel bias random walk
        for (i in 12..14) Q.set(i, i, 1e-5) // Gyro bias random walk
    }

    var linearAccelWorld = doubleArrayOf(0.0, 0.0, 0.0)
        private set
        
    fun predict(accelRaw: FloatArray, gyroRaw: FloatArray, dt: Double) {
        if (dt <= 0) return
        
        val aRaw = SimpleMatrix(3, 1).apply { 
            set(0, 0, accelRaw[0].toDouble())
            set(1, 0, accelRaw[1].toDouble())
            set(2, 0, accelRaw[2].toDouble())
        }
        val wRaw = SimpleMatrix(3, 1).apply { 
            set(0, 0, gyroRaw[0].toDouble())
            set(1, 0, gyroRaw[1].toDouble())
            set(2, 0, gyroRaw[2].toDouble())
        }
        
        val a = aRaw.minus(accelBias)
        val w = wRaw.minus(gyroBias)
        
        val R = getRotationMatrix(quaternion)
        
        // 1. Update nominal state
        val aWorld = R.mult(a).minus(gravity)
        linearAccelWorld[0] = aWorld.get(0,0)
        linearAccelWorld[1] = aWorld.get(1,0)
        linearAccelWorld[2] = aWorld.get(2,0)
        
        position = position.plus(velocity.scale(dt)).plus(aWorld.scale(0.5 * dt * dt))
        velocity = velocity.plus(aWorld.scale(dt))
        quaternion = updateQuaternion(quaternion, w, dt)
        
        // 2. Propagate Covariance
        val F = computeJacobian(R, a, dt)
        P = F.mult(P).mult(F.transpose()).plus(Q)
    }
    
    // Updates the ESKF with a position measurement (e.g., from PDR)
    fun updatePosition(measuredPos: DoubleArray, R_cov: Double) {
        val H = SimpleMatrix(3, 15)
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
        
        val I = SimpleMatrix.identity(15)
        P = I.minus(K.mult(H)).mult(P)
    }
    
    // Zero Velocity Update
    fun updateZUPT(R_cov: Double) {
        val H = SimpleMatrix(3, 15)
        H.set(0, 3, 1.0); H.set(1, 4, 1.0); H.set(2, 5, 1.0)
        
        val R_mat = SimpleMatrix.identity(3).scale(R_cov)
        
        val S = H.mult(P).mult(H.transpose()).plus(R_mat)
        val K = P.mult(H.transpose()).mult(S.invert())
        
        val z = SimpleMatrix(3, 1).minus(velocity) // 0 - nominal velocity
        
        val dx = K.mult(z)
        injectErrorState(dx)
        
        val I = SimpleMatrix.identity(15)
        P = I.minus(K.mult(H)).mult(P)
    }

    private fun injectErrorState(dx: SimpleMatrix) {
        // dx: dp(0..2), dv(3..5), dtheta(6..8), dba(9..11), dbg(12..14)
        position = position.plus(dx.extractMatrix(0, 3, 0, 1))
        velocity = velocity.plus(dx.extractMatrix(3, 6, 0, 1))
        
        val dTheta = dx.extractMatrix(6, 9, 0, 1)
        val dq = axisAngleToQuaternion(dTheta)
        quaternion = multiplyQuaternions(quaternion, dq) // apply rotation error
        
        accelBias = accelBias.plus(dx.extractMatrix(9, 12, 0, 1))
        gyroBias = gyroBias.plus(dx.extractMatrix(12, 15, 0, 1))
    }

    fun getRotationMatrix(q: SimpleMatrix): SimpleMatrix {
        val w = q.get(0, 0); val x = q.get(1, 0); val y = q.get(2, 0); val z = q.get(3, 0)
        val R = SimpleMatrix(3, 3)
        R.set(0, 0, 1 - 2*y*y - 2*z*z); R.set(0, 1, 2*x*y - 2*z*w); R.set(0, 2, 2*x*z + 2*y*w)
        R.set(1, 0, 2*x*y + 2*z*w); R.set(1, 1, 1 - 2*x*x - 2*z*z); R.set(1, 2, 2*y*z - 2*x*w)
        R.set(2, 0, 2*x*z - 2*y*w); R.set(2, 1, 2*y*z + 2*x*w); R.set(2, 2, 1 - 2*x*x - 2*y*y)
        return R
    }
    
    private fun updateQuaternion(q: SimpleMatrix, w: SimpleMatrix, dt: Double): SimpleMatrix {
        val normW = sqrt(w.get(0,0)*w.get(0,0) + w.get(1,0)*w.get(1,0) + w.get(2,0)*w.get(2,0))
        if (normW < 1e-6) return q
        val theta = normW * dt
        val dq = SimpleMatrix(4, 1)
        dq.set(0, 0, cos(theta / 2))
        dq.set(1, 0, w.get(0,0)/normW * sin(theta / 2))
        dq.set(2, 0, w.get(1,0)/normW * sin(theta / 2))
        dq.set(3, 0, w.get(2,0)/normW * sin(theta / 2))
        return multiplyQuaternions(q, dq) 
    }

    private fun multiplyQuaternions(q1: SimpleMatrix, q2: SimpleMatrix): SimpleMatrix {
        val w1 = q1.get(0,0); val x1 = q1.get(1,0); val y1 = q1.get(2,0); val z1 = q1.get(3,0)
        val w2 = q2.get(0,0); val x2 = q2.get(1,0); val y2 = q2.get(2,0); val z2 = q2.get(3,0)
        val q = SimpleMatrix(4, 1)
        q.set(0, 0, w1*w2 - x1*x2 - y1*y2 - z1*z2)
        q.set(1, 0, w1*x2 + x1*w2 + y1*z2 - z1*y2)
        q.set(2, 0, w1*y2 - x1*z2 + y1*w2 + z1*x2)
        q.set(3, 0, w1*z2 + x1*y2 - y1*x2 + z1*w2)
        val norm = sqrt(q.get(0,0)*q.get(0,0) + q.get(1,0)*q.get(1,0) + q.get(2,0)*q.get(2,0) + q.get(3,0)*q.get(3,0))
        return q.scale(1.0 / norm)
    }

    private fun axisAngleToQuaternion(v: SimpleMatrix): SimpleMatrix {
        val norm = sqrt(v.get(0,0)*v.get(0,0) + v.get(1,0)*v.get(1,0) + v.get(2,0)*v.get(2,0))
        val q = SimpleMatrix(4, 1)
        if (norm < 1e-6) {
            q.set(0,0, 1.0); q.set(1,0, 0.0); q.set(2,0, 0.0); q.set(3,0, 0.0)
            return q
        }
        q.set(0, 0, cos(norm / 2))
        q.set(1, 0, v.get(0,0)/norm * sin(norm / 2))
        q.set(2, 0, v.get(1,0)/norm * sin(norm / 2))
        q.set(3, 0, v.get(2,0)/norm * sin(norm / 2))
        return q
    }

    private fun computeJacobian(R: SimpleMatrix, a: SimpleMatrix, dt: Double): SimpleMatrix {
        val F = SimpleMatrix.identity(15)
        F.set(0, 3, dt); F.set(1, 4, dt); F.set(2, 5, dt)
        
        val aWorld = R.mult(a)
        val skewA = skewSymmetric(aWorld)
        val velWrtOri = skewA.scale(-dt)
        F.insertIntoThis(3, 6, velWrtOri)
        
        val velWrtBa = R.scale(-dt)
        F.insertIntoThis(3, 9, velWrtBa)
        
        val oriWrtBg = R.scale(-dt) 
        F.insertIntoThis(6, 12, oriWrtBg)
        
        return F
    }

    private fun skewSymmetric(v: SimpleMatrix): SimpleMatrix {
        val S = SimpleMatrix(3, 3)
        val x = v.get(0,0); val y = v.get(1,0); val z = v.get(2,0)
        S.set(0, 1, -z); S.set(0, 2, y)
        S.set(1, 0, z);  S.set(1, 2, -x)
        S.set(2, 0, -y); S.set(2, 1, x)
        return S
    }
}
