@file:Suppress("unused", "MemberVisibilityCanBePrivate")

package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.experimental.command.Command
import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.RunCommand
import edu.wpi.first.wpilibj.experimental.command.StartEndCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

val closeIntake = InstantCommand(Runnable { Intake.wantsOpen = false })
val openIntake = InstantCommand(Runnable { Intake.wantsOpen = true })

class IntakeHatchCommand(val exhausting: Boolean): FalconCommand(Intake) {

    var wasOpen: Boolean = false

    override fun initialize() {
        println("intaking hatch command")
        Intake.hatchMotorOutput = 1 * exhausting
        Intake.cargoMotorOutput = 0.0
        Intake.wantsOpen = false
        wasOpen = Intake.wantsOpen
        Intake.wantsOpen = false
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        Intake.hatchMotorOutput = 0.0
        Intake.cargoMotorOutput = 0.0
    }

}

class IntakeCargoCommand(val isExhausting: Boolean): FalconCommand(Intake){

    var wasOpen: Boolean = false

    override fun initialize() {
        println("intaking cargo")
        wasOpen = Intake.wantsOpen
        Intake.wantsOpen = !isExhausting

        Intake.hatchMotorOutput = 1.0 * !isExhausting
        Intake.cargoMotorOutput = 1.0 * isExhausting

        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        Intake.cargoMotorOutput = 0.0
        Intake.hatchMotorOutput = 0.0
        super.end(interrupted)
    }
}

private operator fun Number.times(shouldReverse: Boolean) = if(shouldReverse) toDouble() * -1 else toDouble()