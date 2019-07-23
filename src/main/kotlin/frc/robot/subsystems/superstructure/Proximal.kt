package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kProximalCos
import frc.robot.Constants.SuperStructureConstants.kProximalStatic
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.*
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.withSign

object Proximal : ConcurrentFalconJoint<UnboundedRotation, FalconSRX<UnboundedRotation>>() {

    fun resetPosition(ticks: Int) {
        val encoder = synchronized(motor) { motor.encoder }
        encoder.resetPosition(ticks.toDouble())
    }

    override val motor = object : MultiMotorTransmission<UnboundedRotation, FalconSRX<UnboundedRotation>>() {

        override val master: FalconSRX<UnboundedRotation> = FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[0],
                Ports.SuperStructurePorts.ProximalPorts.ROTATION_MODEL)

        override val followers: List<FalconSRX<*>> = listOf(
                FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[1], DefaultNativeUnitModel))

        init {
            if (Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION.size < followers.size)
                throw ArrayIndexOutOfBoundsException("Follower inversion list size contains less indices than the number of followers!")

            master.outputInverted = Ports.SuperStructurePorts.ProximalPorts.TALON_INVERTED
            master.feedbackSensor = Ports.SuperStructurePorts.ProximalPorts.SENSOR
            master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.ProximalPorts.TALON_SENSOR_PHASE)

            followers.forEachIndexed {
                index, followerMotor ->

                followerMotor.follow(master)
                followers[index].talonSRX.setInverted(Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION[index])
            }
            setClosedLoopGains()

            master.talonSRX.configForwardSoftLimitEnable(false)
            master.talonSRX.configReverseSoftLimitEnable(false)
        }

        override fun setClosedLoopGains() {
            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((1785.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(5000)
            master.talonSRX.configMotionSCurveStrength(0)

            master.setClosedLoopGains(
                    0.85, 6.0, ff = 0.45
            )
        }
    }

    fun isWithTolerance(tolerance: Double /* radian */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }

    override fun calculateFeedForward(currentState: JointState) =
            kProximalStatic.withSign(currentState.velocity) + cos(currentState.position) * kProximalCos
}