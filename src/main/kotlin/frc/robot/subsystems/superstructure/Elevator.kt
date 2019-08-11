package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kElevatorRange
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.*

object Elevator : ConcurrentFalconJoint<Meter, FalconSRX<Meter>>() {

    override val motor  = object : MultiMotorTransmission<Meter, FalconSRX<Meter>>() {

        override val master: FalconSRX<Meter> = FalconSRX(ElevatorPorts.TALON_PORTS[0],
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

    init { motor.encoder.position }

    override fun lateInit() { motor.encoder.resetPosition(30.0.inch) }

    /** The maximum distance by which the elevator setpoint can be mutate */
    private val kMaxElevatorOffset = -3.0.inch..3.0.inch

    var elevatorOffset: SIUnit<Meter> = 0.0.meter
        set(newValue) {
            field = newValue.coerceIn(kMaxElevatorOffset)
        }

    override fun customizeWantedState(wantedState: WantedState): WantedState =
            when (wantedState) {
                /** add the [wantedState] and [elevatorOffset] to get an offset total and then bound to the [kElevatorRange] */
                is WantedState.Position<*> -> { ((wantedState as WantedState.Position<Meter>) + elevatorOffset).coerceIn(kElevatorRange) }
                else -> wantedState
            }

    override val currentState get() = motor.currentState

    /** Calculate the arbitrary feed forward given a [currentState] */
    override fun calculateFeedForward(currentState: MultiMotorTransmission.State<Meter>) =
            if (currentState.position > 33.0.inch) 1.2.volt else (-0.72).volt // volts
}
