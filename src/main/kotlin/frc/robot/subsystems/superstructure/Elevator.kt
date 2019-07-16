package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.*
import java.lang.Math.abs

object Elevator : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    val master = FalconSRX(ElevatorPorts.TALON_PORTS[0],
            ElevatorPorts.LENGTH_MODEL)

    private val motor = object : MultiMotorTransmission<Length>(unregisterSubsystem = true) {

        override val master: FalconSRX<Length> = this@Elevator.master

        override val followers: List<FalconSRX<*>> = listOf(
                FalconSRX(ElevatorPorts.TALON_PORTS[1], DefaultNativeUnitModel),
                FalconSRX(ElevatorPorts.TALON_PORTS[2], DefaultNativeUnitModel),
                FalconSRX(ElevatorPorts.TALON_PORTS[3], DefaultNativeUnitModel))

        init {
            master.outputInverted = MASTER_INVERTED
            master.feedbackSensor = ElevatorPorts.SENSOR
            master.talonSRX.setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)

            followers.forEachIndexed {
                index, followerMotor ->

                followerMotor.follow(master)
                followers[index].talonSRX.setInverted(ElevatorPorts.FOLLOWER_INVERSION[index])
            }

            setClosedLoopGains()
        }

        override fun setClosedLoopGains() {
            // TODO also wrap the solenoid boi for the shifter?

            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((4000.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(8000)
            master.talonSRX.configMotionSCurveStrength(0)

            master.setClosedLoopGains(
                    0.45, 4.0, ff = 0.3
            )
        }
    }

    private val innerStageMinLimitSwitch = DigitalInput(0)
    val limitSwitchTriggered: Boolean
        get() = !innerStageMinLimitSwitch.get()

    override fun activateEmergency() = motor.activateEmergency()
    override fun recoverFromEmergency() = motor.recoverFromEmergency()
    override fun setNeutral() {
        wantedState = WantedState.Nothing
        motor.setNeutral()
    }

    override fun lateInit() {
        motor.encoder.resetPosition(0.0)
    }

    private val kMaxElevatorOffset = -3.0 * SILengthConstants.kInchToMeter..3.0 * SILengthConstants.kInchToMeter

    var elevatorOffset: Double = 0.0
        set(newValue) {
            field = newValue.coerceIn(kMaxElevatorOffset)
        }

    fun isWithTolerance(tolerance: Double /* meters */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }

    /* EVERYTHING HERE ON DOWN IS INTENDED TO BE ACCESSED BY COROUTINES */

    val currentState: MultiMotorTransmission.State // = MultiMotorTransmission.State(0.0, 0.0)
        @Synchronized get() = motor.currentState
    var wantedState: WantedState = WantedState.Nothing
        @Synchronized get
        @Synchronized set(newValue) {
            val exception = Exception()
            exception.printStackTrace()

            field = newValue
        }

    override fun updateState() = motor.updateState()

    override fun useState() {
        val wantedState = synchronized(this.wantedState) { this.wantedState }
        val currentState = this.currentState // synchronized(this.currentState) { this.currentState }
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState
                val ff = if (currentState.position > 22.0 * SILengthConstants.kInchToMeter) 1.2 else -0.06 // volts

                synchronized(motor) {
                    motor.setPosition(state.targetPosition + elevatorOffset, ff)
                }
            }
            else -> synchronized(this) { setNeutral() }
        }
    }
}