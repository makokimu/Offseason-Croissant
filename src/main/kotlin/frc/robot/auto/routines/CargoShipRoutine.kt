package frc.robot.auto.routines

import edu.wpi.first.wpilibj.experimental.command.WaitCommand
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeCloseCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.Superstructure
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class CargoShipRoutine(private val mode: CargoShipRoutine.Mode) : AutoRoutine() {

    enum class Mode(
        val path1: TimedTrajectory<Pose2dWithCurvature>,
        val path2: TimedTrajectory<Pose2dWithCurvature>,
        val path3: TimedTrajectory<Pose2dWithCurvature>
    ) {
        SIDE(
            TrajectoryFactory.sideStartToCargoShipS1,
            TrajectoryFactory.cargoShipS1ToLoadingStation,
            TrajectoryFactory.loadingStationToCargoShipS2
        ),
        FRONT(
            TrajectoryFactory.centerStartToCargoShipFL,
            TrajectoryFactory.cargoShipFLToRightLoadingStation,
            TrajectoryFactory.loadingStationToCargoShipFR
        )
    }

    private val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

    override val duration: SIUnit<Second>
        get() = mode.path1.duration + mode.path2.duration + mode.path3.duration

    override val routine
        get() = sequential {

            +parallel {
                +followVisionAssistedTrajectory(mode.path1, pathMirrored, 4.feet, true)
                +sequential {
                    +WaitCommand(mode.path1.duration.second - 3.5.second.second)
                    +Superstructure.kHatchLow.withTimeout(2.0.second)
                }
            }

            val path2 = followVisionAssistedTrajectory(mode.path2, pathMirrored, 4.feet)

            // follow path2 while first outtaking, then close the intake, move the superstructure to the back and grab another hatch
            +parallel {
                +path2
                +sequential {
                    +IntakeHatchCommand(true).withTimeout(0.5.second)
                    +IntakeCloseCommand()
                    +Superstructure.kBackHatchFromLoadingStation
                    +IntakeHatchCommand(false).withExit { path2.isFinished }
                }
            }

            +relocalize(TrajectoryWaypoints.kLoadingStation, false, pathMirrored)

            +parallel {
                +IntakeHatchCommand(false).withTimeout(0.75.second)
                +followVisionAssistedTrajectory(mode.path3, pathMirrored, 4.feet, true)
                +sequential {
//                    +executeFor(2.second, Superstructure.kStowedPosition)
                    +WaitCommand(1.0)
                    +Superstructure.kHatchLow.withTimeout(4.second)
                }
            }

            +parallel {
                +IntakeHatchCommand(true).withTimeout(0.5.second)
                +object : FalconCommand(DriveSubsystem) {
                    override fun execute() {
                        DriveSubsystem.tankDrive(-0.3, -0.3)
                    }

                    override fun end(i: Boolean) {
                        DriveSubsystem.zeroOutputs()
                    }
                }.withTimeout(1.second)
            }
        }
}
