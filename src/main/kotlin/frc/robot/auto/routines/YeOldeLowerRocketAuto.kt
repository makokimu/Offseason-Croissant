package frc.robot.auto.routines

import edu.wpi.first.wpilibj.frc2.command.Command
import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import edu.wpi.first.wpilibj.frc2.command.RunCommand
import edu.wpi.first.wpilibj.frc2.command.WaitCommand
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.auto.paths.TrajectoryFactory.kMaxAcceleration
import frc.robot.auto.paths.TrajectoryFactory.kMaxVelocity
import frc.robot.auto.paths.TrajectoryFactory.kMaxVoltage
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.auto.paths.asWaypoint
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.Superstructure
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.utils.withEquals

class YeOldeLowRocketAuto : AutoRoutine() {

    override val duration: SIUnit<Second>
        get() = 0.second

    var waypoints = listOf(
            TrajectoryWaypoints.kSideStart.asWaypoint(),
            Pose2d(TrajectoryWaypoints.kSideStart.translation + Translation2d(3.5.feet, 0.feet),
                    (0).degree).asWaypoint(),
            Pose2d(12.412.feet, 4.412.feet, -31.615.degree).asWaypoint(),
            TrajectoryFactory.rocketNAdjusted
    )

    val path = TrajectoryFactory.generateTrajectory(
            false,
            waypoints,
            listOf(),
            kMaxVelocity,
            kMaxAcceleration,
            kMaxVoltage
    )

    override val routine = sequential {
        +InstantCommand(Runnable {
            DriveSubsystem.localization.reset((path.firstState.state.pose)) })
        +parallel {
            +followVisionAssistedTrajectory(
                    path,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    4.5.feet)
            +IntakeHatchCommand(false).withTimeout(1.5)
            +sequential {
                +WaitCommand(1.0)
                +Superstructure.everythingMoveTo(19.inch, 0.degree, 4.degree)
            }
        }
        +parallel {
            +IntakeHatchCommand(true).withTimeout(1.0)
            +RunCommand(Runnable{DriveSubsystem.tankDrive(-0.5, -0.5)}).whenFinished {DriveSubsystem.setNeutral()}.withTimeout(1.0)
        }
    }

}