package frc.robot.subsystems.superstructure

import frc.robot.Constants.SuperStructureConstants.kProximalCos
import frc.robot.Constants.SuperStructureConstants.kProximalStatic
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.MultiMotorTransmission
import kotlin.math.cos
import kotlin.math.withSign

object Proximal : MultiMotorTransmission<UnboundedRotation>(
        unregisterSubsystem = false
) {

    var position: Double
        get() {
            val uncomped = encoder.position
//            if(uncomped < Math.toRadians(-90.0)) {
//                if(uncomped < Math.toRadians(-120.0)) return uncomped - Math.toRadians(30.0)
//                else return Math.toRadians(-90.0)
//            }
            return uncomped
        }
        set(newPos) {
            master.motorController.setSelectedSensorPosition(master.model.toNativeUnitPosition(newPos).toInt(), master.encoder.pidIdx, 0)
        }

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
        master.talonSRX.configMotionCruiseVelocity(1785) // about 3500 theoretical max
        master.talonSRX.configMotionAcceleration(5000)
        master.talonSRX.configMotionSCurveStrength(0)

        master.setClosedLoopGains(
                0.85, 6.0, ff = 0.45
        )
    }

    var wantedState: WantedState = WantedState.Nothing

    override fun useState() {
        when (wantedState) {
            is WantedState.Position -> {
                val state = wantedState as WantedState.Position
                val ff = kProximalStatic.withSign(currentState.velocity) + cos(currentState.position) * kProximalCos

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