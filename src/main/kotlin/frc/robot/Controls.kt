package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeCargoCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.SuperStructure
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    var isClimbing = false
        private set

    val driverFalconXbox = xboxController(0) {
        registerEmergencyMode()

//        pov(90).changeOn(TuneElevatorRoutines.tuneKgRoutine)
//        pov(270).changeOn { IntakeSubsystem.badIntakeOffset += .25.inch }
//        pov(90).changeOn { IntakeSubsystem.badIntakeOffset -= .25.inch }

        state({ !isClimbing }) {
            // Vision align
            button(kY).change(VisionDriveCommand(true))
            button(kB).change(VisionDriveCommand(false))

            // Shifting
            button(kA).changeOn { DriveSubsystem.lowGear = true }
            button(kA).changeOff { DriveSubsystem.lowGear = false }

            // Intake
            triggerAxisButton(GenericHID.Hand.kLeft).change(IntakeHatchCommand(true))
            button(kBumperLeft).change(IntakeHatchCommand(false))

            triggerAxisButton(GenericHID.Hand.kRight).change(IntakeCargoCommand(true))
            button(kBumperRight).change(IntakeCargoCommand(false))
        }
    }

    val operatorFalconHID = Joystick(5).mapControls {

        state({ !isClimbing }) {

            // cargo presets
            button(12).changeOn(SuperStructure.kCargoIntake).changeOff { SuperStructure.kStowed.schedule() }
            button(7).changeOn(SuperStructure.kCargoLow).changeOff { SuperStructure.kStowed.schedule() }
            button(6).changeOn(SuperStructure.kCargoMid).changeOff { SuperStructure.kStowed.schedule() }
            button(5).changeOn(SuperStructure.kCargoHigh).changeOff { SuperStructure.kStowed.schedule() }
            button(8).changeOn(SuperStructure.kCargoShip).changeOff { SuperStructure.kStowed.schedule() }

            // hatch presets
            button(3).changeOn(SuperStructure.kHatchLow).changeOff { SuperStructure.kStowed.schedule() }
            button(2).changeOn(SuperStructure.kHatchMid).changeOff { SuperStructure.kStowed.schedule() }
            button(1).changeOn(SuperStructure.kHatchHigh).changeOff { SuperStructure.kStowed.schedule() }

            // that one passthrough preset that doesnt snap back to normal
            button(4).changeOn(SuperStructure.kHatchBackFronLoadingStation)
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