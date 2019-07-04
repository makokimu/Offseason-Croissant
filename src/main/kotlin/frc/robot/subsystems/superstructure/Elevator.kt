package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.MultiMotorTransmission

object Elevator : MultiMotorTransmission<Length>(
        unregisterSubsystem = false
) {

    override val master: FalconSRX<Length> = FalconSRX(ElevatorPorts.TALON_PORTS[0],
            ElevatorPorts.LENGTH_MODEL)

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
        master.talonSRX.configMotionCruiseVelocity(4000) // about 3500 theoretical max
        master.talonSRX.configMotionAcceleration(8000)
        master.talonSRX.configMotionSCurveStrength(0)

        master.setClosedLoopGains(
                0.45, 4.0, ff = 0.3
        )
    }

    var wantedState: WantedState = WantedState.Nothing

    override fun useState() {
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState as WantedState.Position
                val ff = if (currentState.position > 22.0 * SILengthConstants.kInchToMeter) 1.2 else -0.06 // volts

                setPosition(state.targetPosition, ff)
            }
            else -> setNeutral()
        }
    }

    sealed class WantedState {
        object Nothing : WantedState()

        class Position(internal val targetPosition: Double) : WantedState()
    }
}