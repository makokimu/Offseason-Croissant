package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.MultiMotorTransmission
import java.lang.Math.abs

object Wrist : MultiMotorTransmission<UnboundedRotation>(
        unregisterSubsystem = false
) {

    override val master: FalconSRX<UnboundedRotation> = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
            Ports.SuperStructurePorts.WristPorts.ROTATION_MODEL)
        @Synchronized get

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
        master.talonSRX.configMotionCruiseVelocity((2000.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
        master.talonSRX.configMotionAcceleration(3500)
        master.talonSRX.configMotionSCurveStrength(0)

        master.setClosedLoopGains(
                3.5, 0.0, ff = 0.4
        )
    }

    override fun lateInit() {
        position = ((-90).degree.radian)
        Wrist.encoder.resetPosition(Wrist.master.model.toNativeUnitPosition((-90).degree.radian))
        position = ((-90).degree.radian)
        Wrist.encoder.resetPosition(Wrist.master.model.toNativeUnitPosition((-90).degree.radian))
        position = ((-90).degree.radian)
        Wrist.encoder.resetPosition(Wrist.master.model.toNativeUnitPosition((-90).degree.radian))
    }

    var wantedState: WantedState = WantedState.Nothing
        @Synchronized set
        @Synchronized get

    @Synchronized
    override fun useState() {
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState as WantedState.Position
                val ff = 0.0

                synchronized(this) {
                    setPosition(state.targetPosition, ff)
                }
            }
            else -> setNeutral()
        }
    }

    sealed class WantedState {
        object Nothing : WantedState()
        class Position(internal val targetPosition: Double) : WantedState()
    }

    fun isWithTolerance(tolerance: Double /* radian */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }
}