package frc.robot.lib

import org.ghrobotics.lib.motors.FalconMotor

interface MultiMotorTransmission {

    val followerMotors: List<FalconMotor<*>>?

}