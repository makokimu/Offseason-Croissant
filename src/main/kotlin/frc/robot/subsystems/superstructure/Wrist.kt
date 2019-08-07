package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.ConcurrentFalconJoint
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import java.lang.Math.abs

object Wrist : ConcurrentFalconJoint<UnboundedRotation, FalconSRX<UnboundedRotation>>() {

    fun resetPosition(ticks: Int) {
        val encoder = synchronized(motor) { motor.encoder }
        encoder.resetPosition(ticks.toDouble())
    }

    override val motor = object : MultiMotorTransmission<UnboundedRotation, FalconSRX<UnboundedRotation>>() {
        override val master: FalconSRX<UnboundedRotation> = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
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

    override val currentState get() = MultiMotorTransmission.State(Superstructure.currentState.wrist)

    fun isWithTolerance(tolerance: Double /* radian */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }
}