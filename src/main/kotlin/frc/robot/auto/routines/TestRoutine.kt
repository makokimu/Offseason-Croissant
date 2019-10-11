package frc.robot.auto.routines

import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import edu.wpi.first.wpilibj.frc2.command.SequentialCommandGroup
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.units.second

class TestRoutine : AutoRoutine() {

    override val duration = 0.second

//    override val routine = sequential {
//        +InstantCommand(Runnable{
//            DriveSubsystem.robotPosition = TrajectoryFactory.testTrajectory.firstState.state.pose
//            DriveSubsystem.lowGear = false
//        })
//        +DriveSubsystem.followTrajectory(
//                TrajectoryFactory.testTrajectory
//        )
//    }

    override val routine = SequentialCommandGroup(
            InstantCommand(Runnable {
                DriveSubsystem.robotPosition = TrajectoryFactory.testTrajectory.firstState.state.pose
                DriveSubsystem.lowGear = false
            }),
            DriveSubsystem.followTrajectory(
                    TrajectoryFactory.testTrajectory
            ))
}