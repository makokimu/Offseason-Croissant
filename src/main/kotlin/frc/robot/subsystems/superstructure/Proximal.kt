package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kProximalCos
import frc.robot.Constants.SuperStructureConstants.kProximalStatic
import frc.robot.Ports
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.withSign

object Proximal : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    fun resetPosition(ticks: Int) {
        val encoder = synchronized(motor) { motor.encoder }
        encoder.resetPosition(ticks.toDouble())
    }

    val master = FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[0],
            Ports.SuperStructurePorts.ProximalPorts.ROTATION_MODEL)
    private val motor = object : MultiMotorTransmission<UnboundedRotation>(unregisterSubsystem = false) {
        override val master: FalconSRX<UnboundedRotation> = this@Proximal.master

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

    override fun activateEmergency() = motor.activateEmergency()
    override fun recoverFromEmergency() = motor.recoverFromEmergency()
    override fun setNeutral() {
        wantedState = WantedState.Nothing
        motor.setNeutral()
    }

    fun isWithTolerance(tolerance: Double /* radian */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }

    /* EVERYTHING HERE ON DOWN IS INTENDED TO BE ACCESSED BY COROUTINES */

    val currentState: MultiMotorTransmission.State // this is threadsafe maybe
        @Synchronized get() = motor.currentState
    var wantedState: WantedState = WantedState.Nothing
        @Synchronized get
        @Synchronized set

    override fun updateState() = motor.updateState()

    override fun useState() {
        val wantedState = wantedState
        val currentState = motor.currentState
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState
                val ff = kProximalStatic.withSign(currentState.velocity) + cos(currentState.position) * kProximalCos

                synchronized(motor) {
                    motor.setPosition(state.targetPosition, ff)
                }
            }
            else -> synchronized(motor) { setNeutral() }
        }
    }
}