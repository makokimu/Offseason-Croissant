package org.team5940.pantry.lib

import kotlin.math.cos
import kotlin.math.withSign

/**
 * A model using kv-ka-kStatic modeling
 * kV is in <T>
 */
class OblargianTransmissionModel(
    private val kv: Double, // volts/[T]/sec
    private val ka: Double, // volts/[T]/sec/sec
    private val kStatic: Double, // volts
    private val kCos: Double = 0.0 // volts
) {

    fun getFeedForward(position: Double, velocity: Double, accel: Double) =
            kStatic.withSign(velocity) + cos(position) * kCos + kv * velocity + ka * accel

    fun maxSpeedFrom(position: Double, maxVoltage: Double = 10.0): Double {
        val usableVoltage = maxVoltage - kStatic - cos(position) * kCos

        // assuming zero acceleration...
        return kv * usableVoltage // i am speed
    }

    fun getMaxAcceleration(position: Double, maxVoltage: Double = 10.0): Double {

        val usableVoltage = maxVoltage - kStatic - cos(position) * kCos

        return ka * usableVoltage
    }
}