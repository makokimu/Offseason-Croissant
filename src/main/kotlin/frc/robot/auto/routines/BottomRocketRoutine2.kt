package frc.robot.auto.routines

import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import edu.wpi.first.wpilibj.frc2.command.PrintCommand
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports

class BottomRocketRoutine2 : AutoRoutine() {

    private val path1 = TrajectoryFactory.sideStartReversedToRocketFPrepare
    private val path2 = TrajectoryFactory.rocketFPrepareToRocketF

    // Second path goes to the loading station to pick up a hatch panel
    private val path3 = TrajectoryFactory.rocketFToRocketFPrepare
    private val path4 = TrajectoryFactory.rocketFPrepareToLoadingStation

    // Third path goes to the near side of the rocket
    private val path5 = TrajectoryFactory.loadingStationToRocketN

    override val duration: SIUnit<Second>
        get() = path4.duration + path5.duration + path1.duration + path2.duration

    override val routine
        get() = sequential {

            +PrintCommand("Starting")
            +InstantCommand(Runnable { DriveSubsystem.lowGear = false })

            +DriveSubsystem.driveTrajectory(
                    path1,
                    Autonomous.isStartingOnLeft
            )

//            +parallel {
//                +DriveSubsystem.followTrajectory(
//                    path1,
//                    Autonomous.isStartingOnLeft
//                )
//               +sequential {
//                    +DriveSubsystem.notWithinRegion(TrajectoryWaypoints.kHabitatL1Platform)
//                    +WaitCommand(0.5)
//                    +Superstructure.kMatchStartToStowed
//                }//.withTimeout(4.second)
// //                +command
// //                +IntakeHatchCommand(false).withExit { command.isFinished }
//            }

//            +CommandGroupBase.parallel(DriveSubsystem.followTrajectory(
//                    path1,
//                    Autonomous.isStartingOnLeft
//            ), CommandGroupBase.sequence(
//                    DriveSubsystem.notWithinRegion(TrajectoryWaypoints.kHabitatL1Platform),
//                    WaitCommand(0.5),
//                    Superstructure.kMatchStartToStowed)
//            )

//            +PrintCommand("path2")
//
//            +super.followVisionAssistedTrajectory(
//                path2,
//                Autonomous.isStartingOnLeft,
//                4.feet,
//                true
//            )
//
//            // Reorient position on field based on Vision alignment.
//            +relocalize(
//                TrajectoryWaypoints.kRocketF,
//                true,
//                Autonomous.isStartingOnLeft
//            )
//
//            val spline3 = DriveSubsystem.followTrajectory(
//                    path3,
//                    Autonomous.isStartingOnLeft
//            )
//            val spline4 = super.followVisionAssistedTrajectory(
//                    path4,
//                Autonomous.isStartingOnLeft,
//                4.feet, false
//            )
//
//            // Part 2: Place hatch and go to loading station.
//            +parallel {
//                // Follow the trajectory with vision correction to the loading station.
//                +sequential {
//                    +spline3
//                    +spline4
//                }
//                // Take the superstructure to pickup position and arm hatch intake 3 seconds before arrival.
//                +sequential {
//                    // Place hatch panel.
//                    +IntakeHatchCommand(true).withTimeout(1.second)
//                    +WaitCommand(path3.duration.second + path4.duration.second - 3.0)
//                    +IntakeHatchCommand(false).withExit { spline4.isFinished }
//                }
//            }
//
//            // Reorient position on field based on Vision alignment.
//            +relocalize(
//                TrajectoryWaypoints.kLoadingStationReversed,
//                true,
//                Autonomous.isStartingOnLeft
//            )
//
//            // Part 3: Pickup hatch and go to the near side of the rocket.
//            +parallel {
//                // Make sure the intake is holding the hatch panel.
//                +IntakeHatchCommand(false).withTimeout(0.5.second)
//                // Follow the trajectory with vision correction to the near side of the rocket.
//                +super.followVisionAssistedTrajectory(
//                    path5,
//                    Autonomous.isStartingOnLeft,
//                    6.feet, true
//                )
//                // Take the superstructure to scoring height.
//                +Superstructure.kHatchLow.withTimeout(4.second)
//            }
//
//            // Part 4: Score the hatch and go to the loading station for the end of the sandstorm period.
//            +parallel {
//                // Score hatch.
//                // Follow the trajectory to the loading station.
//                +DriveSubsystem.followTrajectory(
//                    TrajectoryFactory.rocketNToLoadingStation,
//                    Autonomous.isStartingOnLeft
//                )
//                // Take the superstructure to a position to pick up the next hatch.
//                +sequential {
//                    +IntakeHatchCommand(releasing = true).withTimeout(0.5.second)
//                    +IntakeCloseCommand()
//                    +Superstructure.kBackHatchFromLoadingStation
//                }
//            }
        }
}