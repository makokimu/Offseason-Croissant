package frc.robot.subsystems.superstructure

import frc.robot.Ports
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.MultiMotorTransmission

object Wrist : MultiMotorTransmission<UnboundedRotation>(
        unregisterSubsystem = false
) {

    override val master: FalconSRX<UnboundedRotation> = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
            Ports.SuperStructurePorts.WristPorts.ROTATION_MODEL)

    init {
        master.outputInverted = Ports.SuperStructurePorts.WristPorts.TALON_INVERTED
        master.feedbackSensor = Ports.SuperStructurePorts.WristPorts.SENSOR
        master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.WristPorts.TALON_SENSOR_PHASE)

        setClosedLoopGains()
    }

    override fun setClosedLoopGains() {
        master.useMotionProfileForPosition = true
        // TODO use FalconSRX properties for velocities and accelerations
        master.talonSRX.configMotionCruiseVelocity(2000,0) // about 3500 theoretical max
        master.talonSRX.configMotionAcceleration(3500)
        master.talonSRX.configMotionSCurveStrength(0)

        master.setClosedLoopGains(
                3.5, 0.0, ff = 0.4
        )
    }
}