package frc.robot.subsystems.superstructure

import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.MultiMotorTransmission

object Wrist : MultiMotorTransmission<UnboundedRotation>(
        unregisterSubsystem = false
) {

    override val master: FalconSRX<UnboundedRotation> = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
            Ports.SuperStructurePorts.WristPorts.ROTATION_MODEL)

    var position: Double
        get() {
            return Proximal.encoder.position
        }
        set(newPos) {

            val pos = Proximal.master.model.toNativeUnitPosition(newPos).toInt()

            println("setting the wrist to $pos ticks!")

            Proximal.master.motorController.setSelectedSensorPosition(pos, Proximal.master.encoder.pidIdx, 0)
        }

    init {
        master.outputInverted = Ports.SuperStructurePorts.WristPorts.TALON_INVERTED
        master.feedbackSensor = Ports.SuperStructurePorts.WristPorts.SENSOR
        master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.WristPorts.TALON_SENSOR_PHASE)

        setClosedLoopGains()
    }

    override fun setClosedLoopGains() {
        master.useMotionProfileForPosition = true
        // TODO use FalconSRX properties for velocities and accelerations
        master.talonSRX.configMotionCruiseVelocity(2000, 0) // about 3500 theoretical max
        master.talonSRX.configMotionAcceleration(3500)
        master.talonSRX.configMotionSCurveStrength(0)

        master.setClosedLoopGains(
                3.5, 0.0, ff = 0.4
        )
    }

    var wantedState: WantedState = WantedState.Nothing
    private var previousState: WantedState = WantedState.Nothing

    override fun useState() {
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState as WantedState.Position
                val ff = 0.0

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