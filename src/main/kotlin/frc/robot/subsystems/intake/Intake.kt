package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.wrappers.FalconMotor
import kotlin.properties.Delegates
import org.ghrobotics.lib.mathematics.units.derivedunits.volt

class Intake(
        val hatchMotor : FalconMotor<Volt> ,
        val cargoMotor : FalconMotor<Volt> ,
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

}