@file:Suppress("unused", "MemberVisibilityCanBePrivate")

package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.experimental.command.Command
import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.RunCommand
import edu.wpi.first.wpilibj.experimental.command.StartEndCommand
import org.ghrobotics.lib.utils.DoubleSource

val closeIntake = InstantCommand(Runnable { Intake.wantsOpen = false })
val openIntake = InstantCommand(Runnable { Intake.wantsOpen = true })

open class RunIntake(cargoSpeed: DoubleSource, hatchSpeed: DoubleSource) : RunCommand(Runnable {
    val isOpen = Intake.wantsOpen
    val cargo = cargoSpeed()
    val hatch = hatchSpeed() * (if (!isOpen) -1 else 1)

    Intake.wantsOpen = isOpen

    Intake.hatchMotorOutput = hatch
    Intake.cargoMotorOutput = cargo
})

class IntakeHatchCommand(exhausting: Boolean) : StartEndCommand(Runnable{Intake.hatchMotorOutput = 1 * exhausting; Intake.wantsOpen = false},
        Runnable{Intake.hatchMotorOutput = 0.0}) {

    var wasOpen: Boolean = false

    override fun initialize() {
        wasOpen = Intake.wantsOpen
        Intake.wantsOpen = false
        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        super.end(interrupted)

    }

}

class IntakeCargoCommand(isExhausting: Boolean) : RunIntake({1.0 * isExhausting}, {1.0 * isExhausting}) {

    var wasOpen: Boolean = false

    override fun initialize() {
        wasOpen = Intake.wantsOpen
        Intake.wantsOpen = true
        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        Intake.wantsOpen = wasOpen
        super.end(interrupted)
    }
}

private operator fun Number.times(shouldReverse: Boolean) = if(shouldReverse) toDouble() * -1 else toDouble()