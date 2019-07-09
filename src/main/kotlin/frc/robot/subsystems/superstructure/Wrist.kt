package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Ports
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.runBlocking
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.FalconChannel
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import java.lang.Math.abs

object Wrist : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    fun resetPosition(ticks: Int) {
        val encoder = synchronized(motor) { motor.encoder }
        encoder.resetPosition(ticks.toDouble())
    }

    val     master = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
            Ports.SuperStructurePorts.WristPorts.ROTATION_MODEL)
    private val motor = object: MultiMotorTransmission<UnboundedRotation>(
            unregisterSubsystem = false
    ) {
        override val master: FalconSRX<UnboundedRotation> = this@Wrist.master

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

    override fun activateEmergency() = motor.activateEmergency()
    override fun recoverFromEmergency() = motor.recoverFromEmergency()

    fun isWithTolerance(tolerance: Double /* radian */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }

    /* EVERYTHING HERE ON DOWN IS INTENDED TO BE ACCESSED BY COROUTINES */

    val currentState: MultiMotorTransmission.State // this is threadsafe maybe by virtue of being a call to currentStateChannel
        @Synchronized get() = motor.currentState
    var wantedState: WantedState = WantedState.Nothing
        @Synchronized get
        @Synchronized set

    override fun updateState() = motor.updateState()

    override fun useState() {
        val wantedState = synchronized(this.wantedState) {this.wantedState}
//        val currentState = synchronized(this.currentState) { this.currentState }
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState
                val ff = 0.0

                synchronized(motor) {
                    motor.setPosition(state.targetPosition, ff)
                }
            }
            else -> synchronized(motor) { setNeutral() }
        }
    }
}