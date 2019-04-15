package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.wrappers.FalconMotor
import kotlin.properties.Delegates
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import frc.robot.Ports.kPCMID
import frc.robot.Ports.IntakePorts.SHIFTER_PORTS
import frc.robot.Ports.IntakePorts.CARGO_PORT
import frc.robot.Ports.IntakePorts.HATCH_PORT
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

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
        hatchMotor.percentOutput = demand.value
    }

    fun setCargoSpeed(demand : Volt) {
        cargoMotor.velocity = demand.value
    }

    companion object {
        fun createRealIntake() : Intake {
            val hatchMotor = NativeFalconSRX(HATCH_PORT)
            val cargoMotor = NativeFalconSRX(CARGO_PORT)
            val solenoid = DoubleSolenoid(kPCMID, SHIFTER_PORTS[0], SHIFTER_PORTS[1])

            return Intake(hatchMotor, cargoMotor, solenoid)
        }
    }

}