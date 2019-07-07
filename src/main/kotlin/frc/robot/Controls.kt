package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.experimental.command.ConditionalCommand
import edu.wpi.first.wpilibj.experimental.command.PrintCommand
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeCargoCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.Superstructure
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.wrappers.hid.*
import org.team5940.pantry.lib.greaterThanAxisButton
import org.team5940.pantry.lib.lessThanAxisButton
import java.util.function.BooleanSupplier

object Controls {

    var isClimbing = false
        private set

    val driverFalconXbox = xboxController(0) {
        registerEmergencyMode()

        state({ !isClimbing }) {
            // Vision align
            triggerAxisButton(GenericHID.Hand.kRight).change(
                    ConditionalCommand(VisionDriveCommand(true), VisionDriveCommand(false),
                            BooleanSupplier { !Superstructure.currentState.isPassedThrough }))

            // Shifting
            button(kBumperLeft).changeOn { DriveSubsystem.lowGear = true }.changeOff { DriveSubsystem.lowGear = false }
        }
    }

    private val operatorJoy = Joystick(5)
    val operatorFalconHID = operatorJoy.mapControls {

        state({ !isClimbing }) {

            // cargo presets
            button(12).changeOn(Superstructure.kCargoIntake).changeOff { Superstructure.kStowed.schedule() }
            button(7).changeOn(Superstructure.kCargoLow).changeOff { Superstructure.kStowed.schedule() }
            button(6).changeOn(Superstructure.kCargoMid).changeOff { Superstructure.kStowed.schedule() }
            button(5).changeOn(Superstructure.kCargoHigh).changeOff { Superstructure.kStowed.schedule() }
            button(8).changeOn(Superstructure.kCargoShip).changeOff { Superstructure.kStowed.schedule() }

            // hatch presets
            button(3).changeOn(Superstructure.kHatchLow).changeOff { Superstructure.kStowed.schedule() }
            button(2).changeOn(Superstructure.kHatchMid).changeOff { Superstructure.kStowed.schedule() }
            button(1).changeOn(Superstructure.kHatchHigh).changeOff { Superstructure.kStowed.schedule() }

            // that one passthrough preset that doesnt snap back to normal
            button(4).changeOn(Superstructure.kBackHatchFromLoadingStation)

            // hatches
            lessThanAxisButton(operatorJoy, 1).change(IntakeHatchCommand(releasing = false))
            greaterThanAxisButton(operatorJoy, 1).change(IntakeHatchCommand(releasing = true))

            // cargo -- intake is a bit tricky, it'll go to the intake preset automatically
            // the lessThanAxisButton represents "intaking", and the greaterThanAxisButton represents "outtaking"
            val cargoCommand = sequential { +PrintCommand("running cargoCommand"); +Superstructure.kCargoIntake; +IntakeCargoCommand(releasing = false) }
            lessThanAxisButton(operatorJoy, 0).changeOff { Superstructure.kStowed.schedule() }.change(cargoCommand)
            greaterThanAxisButton(operatorJoy,0).changeOff { Superstructure.kStowed.schedule() }.change(IntakeCargoCommand(true))

        }
    }

    fun update() {
        driverFalconXbox.update()
        operatorFalconHID.update()
    }
}


private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn {
        Robot.activateEmergency()
    }
    button(kStart).changeOn {
        Robot.recoverFromEmergency()
    }
}