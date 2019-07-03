package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeCargoCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
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
            button(kY).change(VisionDriveCommand(VisionDriveCommand.TargetSide.FRONT))
            button(kB).change(VisionDriveCommand(VisionDriveCommand.TargetSide.BACK))

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

}

private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn {
        Robot.activateEmergency()
    }
    button(kStart).changeOn {
        Robot.recoverFromEmergency()
    }
}