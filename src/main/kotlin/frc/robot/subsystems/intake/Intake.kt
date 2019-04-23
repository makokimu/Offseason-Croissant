package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import kotlin.properties.Delegates
import frc.robot.Ports.kPCMID
import frc.robot.Ports.IntakePorts.SHIFTER_PORTS
import frc.robot.Ports.IntakePorts.CARGO_PORT
import frc.robot.Ports.IntakePorts.HATCH_PORT
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

class Intake(
        val hatchMotor : FalconMotor<NativeUnit>,
        val cargoMotor : FalconMotor<NativeUnit>,
        private val clampPistons : FalconSolenoid
) {

    // Open and close the intake
    var intakeOpen : Boolean by Delegates.observable(false) { _, _, wantsClosed ->
        if (wantsClosed) {
            clampPistons.state = FalconSolenoid.State.Forward
        } else {
            clampPistons.state = FalconSolenoid.State.Reverse
        }
    }

    fun setHatchSpeed(demand : Volt) {
        hatchMotor.setDutyCycle(demand.value / 12)
    }

    fun setCargoSpeed(demand : Volt) {
        cargoMotor.setDutyCycle(demand.value / 12)
    }

    companion object {
        fun createRealIntake() : Intake {
            val hatchMotor = FalconSRX(HATCH_PORT, DefaultNativeUnitModel)
            val cargoMotor = FalconSRX(CARGO_PORT, DefaultNativeUnitModel)
            val solenoid = FalconDoubleSolenoid(SHIFTER_PORTS[0], SHIFTER_PORTS[1], kPCMID)

            return Intake(hatchMotor, cargoMotor, solenoid)
        }
    }

}