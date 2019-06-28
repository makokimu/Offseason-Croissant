package frc.robot.subsystems.superstructure

import frc.robot.Ports
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.MultiMotorTransmission

object Proximal : MultiMotorTransmission<UnboundedRotation>(
        unregisterSubsystem = false
) {

    override val master: FalconSRX<UnboundedRotation> = FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[0],
            Ports.SuperStructurePorts.ProximalPorts.ROTATION_MODEL)

    override val followers: List<FalconSRX<*>> = listOf(
            FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[1], DefaultNativeUnitModel))

    init {

        if(Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION.size < followers.size)
            throw ArrayIndexOutOfBoundsException("Follower inversion list size contains less indices than the number of followers!")

        master.outputInverted = true
        master.feedbackSensor = Ports.SuperStructurePorts.ProximalPorts.SENSOR
        master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.ProximalPorts.TALON_SENSOR_PHASE)

        followers.forEachIndexed {
            index, followerMotor ->

            followerMotor.follow(master)
            followers[index].talonSRX.setInverted(Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION[index])
        }

        setClosedLoopGains()
    }

    override fun setClosedLoopGains() {
        master.useMotionProfileForPosition = true
        // TODO use FalconSRX properties for velocities and accelerations
        master.talonSRX.configMotionCruiseVelocity(1785) // about 3500 theoretical max
        master.talonSRX.configMotionAcceleration(6000)
        master.talonSRX.configMotionSCurveStrength(0)

        master.setClosedLoopGains(
                0.85, 6.0, ff = 0.45
        )
    }
}