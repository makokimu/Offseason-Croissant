package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kElevatorRange
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.*
import java.lang.Math.abs

object Elevator : ConcurrentFalconJoint<Length, FalconSRX<Length>>() {

    override val motor = object : MultiMotorTransmission<Length, FalconSRX<Length>>() {

        override val master: FalconSRX<Length> = FalconSRX(ElevatorPorts.TALON_PORTS[0],
                ElevatorPorts.LENGTH_MODEL)

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
    val limitSwitchTriggered: Boolean get() = !innerStageMinLimitSwitch.get()

    override fun lateInit() { motor.encoder.resetPosition(0.0) }

    /** The maximum distance by which the elevator setpoint can be mutate */
    private val kMaxElevatorOffset = -3.0 * SILengthConstants.kInchToMeter..3.0 * SILengthConstants.kInchToMeter

    var elevatorOffset: Double = 0.0
        set(newValue) {
            field = newValue.coerceIn(kMaxElevatorOffset)
        }

    override fun customizeWantedState(wantedState: WantedState): WantedState =
            when (wantedState) {
                /** add the [wantedState] and [elevatorOffset] to get an offset total and then bound to the [kElevatorRange] */
                is WantedState.Position -> { (wantedState + elevatorOffset).coerceIn(kElevatorRange) }
                else -> wantedState
            }

    fun isWithTolerance(tolerance: Double /* meters */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }

    /** Calculate the arbitrary feed forward given a [currentState] */
    override fun calculateFeedForward(currentState: MultiMotorTransmission.State) =
            if (currentState.position > 22.0 * SILengthConstants.kInchToMeter) 1.2 else -0.72 // volts
}
