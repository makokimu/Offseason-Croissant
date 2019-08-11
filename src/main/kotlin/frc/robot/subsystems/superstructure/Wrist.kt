package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.ConcurrentFalconJoint
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import java.lang.Math.abs

object Wrist : ConcurrentFalconJoint<Radian, FalconSRX<Radian>>() {

    fun resetPosition(ticks: Int) {
        val encoder = synchronized(motor) { motor.encoder }
        encoder.resetPositionRaw(ticks.toDouble().nativeUnits)
    }

    override val motor = object : MultiMotorTransmission<Radian, FalconSRX<Radian>>() {
        override val master: FalconSRX<Radian> = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
                Ports.SuperStructurePorts.WristPorts.ROTATION_MODEL)

        init {
            master.outputInverted = Ports.SuperStructurePorts.WristPorts.TALON_INVERTED
            master.feedbackSensor = Ports.SuperStructurePorts.WristPorts.SENSOR
            master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.WristPorts.TALON_SENSOR_PHASE)

            setClosedLoopGains()
        }

        override fun setClosedLoopGains() {
            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((2000.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(3500)
            master.talonSRX.configMotionSCurveStrength(0)

            master.setClosedLoopGains(
                    3.5, 0.0, ff = 0.4
            )
        }
    }
}