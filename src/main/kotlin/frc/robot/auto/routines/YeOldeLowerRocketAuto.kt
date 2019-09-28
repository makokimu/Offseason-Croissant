package frc.robot.auto.routines

import edu.wpi.first.wpilibj.frc2.command.Command
import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.auto.paths.TrajectoryFactory.kMaxAcceleration
import frc.robot.auto.paths.TrajectoryFactory.kMaxVelocity
import frc.robot.auto.paths.TrajectoryFactory.kMaxVoltage
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.auto.paths.asWaypoint
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.Superstructure
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.utils.withEquals

class YeOldeLowRocketAuto : AutoRoutine() {

    override val duration: SIUnit<Second>
        get() = 0.second

    var waypoints = listOf(
            TrajectoryWaypoints.kSideStart.asWaypoint(),
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
            DriveSubsystem.localization.reset((path.firstState.state.pose.mirror)) })

        +parallel {
            +Superstructure.everythingMoveTo(19.inch, 0.degree, 4.degree)
            +followVisionAssistedTrajectory(
                    path,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    3.feet)
        }
    }

}