package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.frc2.command.*
import frc.robot.auto.routines.withExit
import frc.robot.subsystems.climb.ClimbSubsystem
//import frc.robot.subsystems.climb.SketchyTest
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeCargoCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.*
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports
import org.team5940.pantry.lib.Updatable
import org.team5940.pantry.lib.WantedState
import java.util.function.BooleanSupplier

object Controls : Updatable {

    var isClimbing = false
        set

    private val zero = ZeroSuperStructureRoutine()

    val driverControllerLowLevel = XboxController(0)
    val driverFalconXbox = driverControllerLowLevel.mapControls {
        registerEmergencyMode()

//        button(kB).changeOn { isClimbing = true }
//        button(kX).changeOn { isClimbing = false }
        button(kY).changeOn(ClimbSubsystem.fullS3ndClimbCommand)

        state({ !isClimbing }) {
            // Vision align
            triggerAxisButton(GenericHID.Hand.kRight).change(
                    ConditionalCommand(VisionDriveCommand(true), VisionDriveCommand(false),
                            BooleanSupplier { !Superstructure.currentState.isPassedThrough }))

            // Shifting
            button(kBumperLeft).changeOn { DriveSubsystem.lowGear = true }.changeOff { DriveSubsystem.lowGear = false }
            button(kB).changeOn(ClimbSubsystem.prepMove)
        }

    }

//    val auxXbox = XboxController(1)
//    val auxFalconXbox = auxXbox.mapControls {
//        button(kY).changeOn(ClimbSubsystem.fullS3ndClimbCommand)
//    }

    val operatorJoy = Joystick(5)
    val operatorFalconHID = operatorJoy.mapControls {

//        button(4).changeOn(ClimbSubsystem.fullS3ndClimbCommand)

        state({ !isClimbing }) {

            // elevator jogging

            // cargo presets
//            button(12).changeOn(Superstructure.kCargoIntake.andThen { Intake.wantsOpen = true }) // .changeOff { Superstructure.kStowed.schedule() }
            button(7).changeOn(Superstructure.kCargoLow) // .changeOff { Superstructure.kStowed.schedule() }
            button(6).changeOn(Superstructure.kCargoMid) // .changeOff { Superstructure.kStowed.schedule() }
            button(5).changeOn(Superstructure.kCargoHigh) // .changeOff { Superstructure.kStowed.schedule() }
            button(8).changeOn(Superstructure.kCargoShip) // .changeOff { Superstructure.kStowed.schedule() }

            // hatch presets
            button(3).changeOn(Superstructure.kHatchLow) // .changeOff { Superstructure.kStowed.schedule() }
            button(2).changeOn(Superstructure.kHatchMid) // .changeOff { Superstructure.kStowed.schedule() }
            button(1).changeOn(Superstructure.kHatchHigh) // .changeOff { Superstructure.kStowed.schedule() }

            // Stow (for now like this coz i dont wanna break anything
            button(10).changeOn(Superstructure.kStowed)

            button(9).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position + 1.inch })
            button(11).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position - 1.inch })

            // that one passthrough preset that doesnt snap back to normal
            button(4).changeOn(Superstructure.kBackHatchFromLoadingStation)

            // hatches
            lessThanAxisButton(1).change(IntakeHatchCommand(releasing = false))
            greaterThanAxisButton(1).change(IntakeHatchCommand(releasing = true))

            // cargo -- intake is a bit tricky, it'll go to the intake preset automatically
            // the lessThanAxisButton represents "intaking", and the greaterThanAxisButton represents "outtaking"
            val cargoCommand = sequential { +PrintCommand("running cargoCommand"); +Superstructure.kCargoIntake; +IntakeCargoCommand(releasing = false) }
            lessThanAxisButton(0).changeOff { (sequential{ +ClosedLoopWristMove(40.degree) ; +Superstructure.kStowed;  }).schedule() }.change(cargoCommand)
            greaterThanAxisButton(0).changeOff { Superstructure.kStowed.schedule() }.change(IntakeCargoCommand(true))
        }
    }

    override fun update() {
        driverFalconXbox.update()
        operatorFalconHID.update()
//        auxFalconXbox.update()
    }
}

private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }

private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn {
        Robot.activateEmergency()
    }
    button(kStart).changeOn {
        Robot.recoverFromEmergency()
    }
}