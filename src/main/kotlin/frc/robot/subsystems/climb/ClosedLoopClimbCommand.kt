package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import edu.wpi.first.wpilibj.frc2.command.RunCommand
import frc.robot.auto.routines.AutoRoutine
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.*
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second
import org.team5940.pantry.lib.WantedState

class ClosedLoopClimbCommand: AutoRoutine() {

    override val duration = 8.second

    override val routine = sequential {
        // take superstructure up to scoring height
        +SuperstructurePlanner.everythingMoveTo(30.inch, 0.degree, 0.degree) // TODO check preset
        // yote forward
        +InstantCommand(Runnable { DriveSubsystem.lowGear = true })
        +executeFor(2.second, object : FalconCommand(DriveSubsystem) {
            override fun execute() {
                DriveSubsystem.tankDrive(0.5, 0.5)
            }
        })
        // take superstructure to prop up position
        +SuperstructurePlanner.everythingMoveTo(30.inch, (-60).degree, 120.degree) // TODO check preset
        +parallel(
                object : FalconCommand(ClimbSubsystem, Elevator) {
                    var elevatorCruisingToPosition = true
                    override fun execute() {
                        // something something make the speeds the same between the elevator and climb somehow?
                        ClimbSubsystem.wantedState = WantedState.Position(18.inch)
                        // do the Thing with the Elevator
                        if(Elevator.currentState.position < 35.inch && elevatorCruisingToPosition) // TODO check preset
                            elevatorCruisingToPosition = false
                        Elevator.wantedState = if(elevatorCruisingToPosition) WantedState.Voltage((-4).volt) else
                            WantedState.Position(33.inch) // TODO check preset
                    }

                    override fun isFinished() = Elevator.isWithTolerance(2.inch)
                            && ClimbSubsystem.isWithTolerance(2.inch)
                }
        )
        // yeet forward again
        +executeFor(5.second, object : FalconCommand(DriveSubsystem, ClimbSubsystem) {
            override fun execute() {
                DriveSubsystem.tankDrive(0.5, 0.5)
                ClimbSubsystem.intakeWheels.setDutyCycle(0.5)
            }
        })
        // stow the elevator somehow
        +ClosedLoopElevatorMove(30.inch)
        +ClosedLoopWristMove(45.degree)
        +ClosedLoopProximalMove((-60).degree)
    }

}