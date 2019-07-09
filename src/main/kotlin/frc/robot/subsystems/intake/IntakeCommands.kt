@file:Suppress("unused", "MemberVisibilityCanBePrivate")

package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.abs

val closeIntake = InstantCommand(Runnable { Intake.wantsOpen = false })
val openIntake = InstantCommand(Runnable { Intake.wantsOpen = true })

class IntakeHatchCommand(val releasing: Boolean) : FalconCommand(Intake) {

    var wasOpen: Boolean = false

    override fun initialize() {
        println("intaking hatch command")
        Intake.hatchMotorOutput = 1 * releasing
        Intake.cargoMotorOutput = 0.0
        Intake.wantsOpen = false
        wasOpen = Intake.wantsOpen
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        Intake.hatchMotorOutput = 0.0
        Intake.cargoMotorOutput = 0.0
    }
}

class IntakeCargoCommand(val releasing: Boolean) : FalconCommand(Intake) {

    var wasOpen: Boolean = false

    override fun initialize() {
        println("${if (releasing) "releasing" else "intaking"} cargo command!")
        wasOpen = Intake.wantsOpen
        Intake.wantsOpen = !releasing

        Intake.hatchMotorOutput = 1.0 * !releasing
        Intake.cargoMotorOutput = 1.0 * releasing

        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        Intake.cargoMotorOutput = 0.0
        Intake.hatchMotorOutput = 0.0
        super.end(interrupted)
    }
}

class IntakeTeleopCommand : FalconCommand(Intake) {

    override fun execute() {
        val cargoSpeed = -cargoSource()
        val hatchSpeed = -hatchSource()

        if (abs(cargoSpeed) > 0.2) {
            Intake.hatchMotorOutput = -1.0 * cargoSpeed
            Intake.cargoMotorOutput = cargoSpeed
        } else {
            Intake.hatchMotorOutput = hatchSpeed
            Intake.cargoMotorOutput = 0.0
        }
    }

    override fun end(interrupted: Boolean) {
        Intake.hatchMotorOutput = 0.0
        Intake.cargoMotorOutput = 0.0
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

private operator fun Number.times(shouldReverse: Boolean) = if (shouldReverse) toDouble() * -1 else toDouble()