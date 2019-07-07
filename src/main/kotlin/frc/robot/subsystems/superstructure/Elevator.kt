package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.boundTo
import java.lang.Math.abs

object Elevator : MultiMotorTransmission<Length>(
        unregisterSubsystem = false
) {

    override val master: FalconSRX<Length> = FalconSRX(ElevatorPorts.TALON_PORTS[0],
            ElevatorPorts.LENGTH_MODEL)
        @Synchronized get


    override val followers: List<FalconSRX<*>> = listOf(
            FalconSRX(ElevatorPorts.TALON_PORTS[1], DefaultNativeUnitModel),
            FalconSRX(ElevatorPorts.TALON_PORTS[2], DefaultNativeUnitModel),
            FalconSRX(ElevatorPorts.TALON_PORTS[3], DefaultNativeUnitModel))

    private val innerStageMinLimitSwitch = DigitalInput(0)
    val limitSwitchTriggered: Boolean
        get() = !innerStageMinLimitSwitch.get()

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

    var elevatorOffset: Double = 0.0
        set(newvalue) {

            field = newvalue.boundTo(-3.0 / SILengthConstants.kInchToMeter, 3.0 / SILengthConstants.kInchToMeter)
        }

    var wantedState: WantedState = WantedState.Nothing
        @Synchronized set
        @Synchronized get

    @Synchronized
    override fun useState() {
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState as WantedState.Position
                val ff = if (currentState.position > 22.0 * SILengthConstants.kInchToMeter) 1.2 else -0.06 // volts

                synchronized(this) {
                    setPosition(state.targetPosition + elevatorOffset, ff)
                }

            }
            else -> setNeutral()
        }
    }

    sealed class WantedState {
        object Nothing : WantedState()

        class Position(internal val targetPosition: Double) : WantedState()
    }

    fun isWithTolerance(tolerance: Double /* meters */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }
}