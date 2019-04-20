package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import kotlin.properties.Delegates
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import frc.robot.Ports.kPCMID
import frc.robot.Ports.IntakePorts.SHIFTER_PORTS
import frc.robot.Ports.IntakePorts.CARGO_PORT
import frc.robot.Ports.IntakePorts.HATCH_PORT
import frc.robot.lib.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX

class Intake(
        val hatchMotor : FalconMotor<NativeUnit>,
        val cargoMotor : FalconMotor<NativeUnit>,
        private val clampPistons : DoubleSolenoid
) {

    // Open and close the intake
    var intakeOpen : Boolean by Delegates.observable(false) { _, _, wantsClosed ->
        if (wantsClosed) {
            clampPistons.set(kForward)
        } else {
            clampPistons.set(kReverse)
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
            val solenoid = DoubleSolenoid(kPCMID, SHIFTER_PORTS[0], SHIFTER_PORTS[1])

            return Intake(hatchMotor, cargoMotor, solenoid)
        }
    }

}