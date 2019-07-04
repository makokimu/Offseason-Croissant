package frc.robot.subsystems.intake

import kotlin.properties.Delegates
import frc.robot.Ports.kPCMID
import frc.robot.Ports.IntakePorts.CARGO_PORT
import frc.robot.Ports.IntakePorts.HATCH_PORT
import frc.robot.Ports.IntakePorts.PISTON_PORTS
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {

    private val hatchMotor = FalconSRX(HATCH_PORT, DefaultNativeUnitModel)
    private val cargoMotor = FalconSRX(CARGO_PORT, DefaultNativeUnitModel)
    private val solenoid = FalconDoubleSolenoid(PISTON_PORTS[0], PISTON_PORTS[1], kPCMID)

    // Open and close the intake
    var wantsOpen: Boolean by Delegates.observable(false) { _, _, wantsClosed ->
        if (wantsClosed) {
            solenoid.state = FalconSolenoid.State.Forward
        } else {
            solenoid.state = FalconSolenoid.State.Reverse
        }
    }

    init {
        hatchMotor.configCurrentLimit(true, FalconSRX.CurrentLimitConfig(35.amp, 400.millisecond, 20.amp))
        cargoMotor.configCurrentLimit(true, FalconSRX.CurrentLimitConfig(35.amp, 400.millisecond, 20.amp))
        cargoMotor.talonSRX.configPeakOutputForward(0.8)
        cargoMotor.talonSRX.configPeakOutputReverse(-0.8)
    }

    var hatchMotorOutput: Double
        get() = hatchMotor.voltageOutput / 12.0
        set(value) = hatchMotor.setDutyCycle(value)

    var cargoMotorOutput: Double
        get() = cargoMotor.voltageOutput / 12.0
        set(value) = cargoMotor.setDutyCycle(value)
}