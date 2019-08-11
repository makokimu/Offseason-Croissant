@file:Suppress("unused", "MemberVisibilityCanBePrivate")

package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.volt
import kotlin.math.abs

val closeIntake = InstantCommand(Runnable { Intake.wantsOpen = false })
val openIntake = InstantCommand(Runnable { Intake.wantsOpen = true })

class IntakeHatchCommand(val releasing: Boolean) : FalconCommand(Intake) {

    var wasOpen: Boolean = false

    override fun initialize() {
        println("intaking hatch command")
        Intake.hatchMotorOutput = 12.volt * (if(releasing) 1 else -1)
        Intake.cargoMotorOutput = 0.volt
        Intake.wantsOpen = false
        wasOpen = Intake.wantsOpen
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        Intake.hatchMotorOutput = 0.volt
        Intake.cargoMotorOutput = 0.volt
    }
}

class IntakeCargoCommand(val releasing: Boolean) : FalconCommand(Intake) {

    var wasOpen: Boolean = false

    override fun initialize() {
        println("${if (releasing) "releasing" else "intaking"} cargo command!")
        wasOpen = Intake.wantsOpen
        Intake.wantsOpen = !releasing

        Intake.hatchMotorOutput = 12.volt * (if(!releasing) 1 else -1)
        Intake.cargoMotorOutput = 12.volt * (if(releasing) 1 else -1)

        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        Intake.cargoMotorOutput = 0.volt
        Intake.hatchMotorOutput = 0.volt
        super.end(interrupted)
    }
}

class IntakeTeleopCommand : FalconCommand(Intake) {

    override fun execute() {
        val cargoSpeed = -cargoSource()
        val hatchSpeed = -hatchSource()

        if (abs(cargoSpeed) > 0.2) {
            Intake.hatchMotorOutput = (-12).volt * cargoSpeed
            Intake.cargoMotorOutput = 12.volt * cargoSpeed
        } else {
            Intake.hatchMotorOutput = 12.volt * hatchSpeed
            Intake.cargoMotorOutput = 0.volt
        }
    }

    override fun end(interrupted: Boolean) {
        Intake.hatchMotorOutput = 0.volt
        Intake.cargoMotorOutput = 0.volt
    }

    companion object {
        val cargoSource by lazy { Controls.operatorFalconHID.getRawAxis(0) }
        val hatchSource by lazy { Controls.operatorFalconHID.getRawAxis(1) }
    }
}

class IntakeCloseCommand : FalconCommand(Intake) {
    init {
        Intake.wantsOpen = false
    }
}
