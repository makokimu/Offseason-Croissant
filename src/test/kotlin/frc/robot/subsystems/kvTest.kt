package frc.robot.subsystems

import com.team254.lib.physics.DCMotorTransmission
import frc.robot.Constants
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.kInchToMeter
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.PI

fun main() {

    // kv and ka calculation
    val G: Double = 5.0 // output over input
    val motorResistance = 0.114 // ohms?
    val radius = 1.5 * kInchToMeter / 2.0 // radius, meters
    val m = 45.0 // kilogram
    val motorCount = 4.0
    val internalKv = 50.40 // volts per meter per sec
    val internalKt = 0.0247 // volts per meter per sec squared
    val metersKa = (motorResistance * radius * m)/(G * internalKt)
    val metersKv = (G * G * internalKt * metersKa) / (motorResistance * radius * radius * m * internalKv)

    val stiltTransmission = DCMotorTransmission(
            1 / (metersKv * 1 / (2.0 / radius)), // 603.2 /* rad per sec free */ / R / 12.0, // 42:1 gearing
            radius * radius * m / (2.0 * metersKa / (2.0 / radius)),
//            1.5.inch.meter / 2.0 * 1.5.inch.meter / 2.0 * Constants.DriveConstants.kRobotMass
//                    / 2.0 / (2.0 * 0.164 /* volts per meter per second squared (FRCControl) */ * 1 /
//                    ( /* radians per meter */ 2.0 / 1.5.inch.meter ) ), // 42:1 gearing given by frccontrol (1.01?)
            0.0 // totally a guess
    )

    val stiltTransmissio2 = DCMotorTransmission(
            603.2 /* rad per sec free */ / G / 12.0, // 42:1 gearing
//            r * r * m / (2.0 * metersKa / (2.0 / r)),
            1.5.inch.meter / 2.0 * 1.5.inch.meter / 2.0 * Constants.DriveConstants.kRobotMass
                    / 2.0 / (2.0 * 0.164 /* volts per meter per second squared (FRCControl) */ * 1 /
                    ( /* radians per meter */ 2.0 / 1.5.inch.meter ) ), // 42:1 gearing given by frccontrol (1.01?)
            0.0 // totally a guess
    )

//    println("State space transmission: speedPerVolt ${stiltTransmission.speedPerVolt} torque ${radius * radius * m / (2.0 * metersKa / (2.0 / radius))}")
//    println("other transmission: speedPerVolt ${stiltTransmissio2.speedPerVolt} torque ${1.5.inch.meter / 2.0 * 1.5.inch.meter / 2.0 * Constants.DriveConstants.kRobotMass
//            / 2.0 / (2.0 * 0.164 /* volts per meter per second squared (FRCControl) */ * 1 /
//            ( /* radians per meter */ 2.0 / 1.5.inch.meter ) )}")

    val motorSpeedRadPerSec = (0.3 / (1.5.inch.meter * Math.PI) * 2 * Math.PI) // meters per sec divided by meter per circum is revolutions per sec, times 2pi is rad per sec

    println(stiltTransmission.getVoltageForTorque(motorSpeedRadPerSec, 0.6))

}