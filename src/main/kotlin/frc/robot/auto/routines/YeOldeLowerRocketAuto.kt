package frc.robot.auto.routines

import edu.wpi.first.wpilibj.frc2.command.Command
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
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class YeOldeLowRocketAuto : AutoRoutine() {

    override val duration: SIUnit<Second>
        get() = 0.second

    var fallOFfHab = listOf(
            TrajectoryWaypoints.kSideStart.asWaypoint(),
            Pose2d((8).feet,
                    (18.5).feet,
                    (40).degree).mirror.asWaypoint()

    )

    var floorToRocketC = listOf(
            Pose2d((8.0).feet,
                    (18.5).feet,
                    (40).degree).mirror.asWaypoint(),
//            Pose2d((14.14).feet,
//                    (24.5).feet,
//                    (20).degree).asWaypoint()
            TrajectoryFactory.rocketNAdjusted
    )

    val t_fallOFfHab = TrajectoryFactory.generateTrajectory(
            false,
            fallOFfHab,
            listOf(),
            kMaxVelocity,
            kMaxAcceleration,
            kMaxVoltage
    )

    val t_floorToRocketC = TrajectoryFactory.generateTrajectory(
            false, floorToRocketC,
            TrajectoryFactory.getConstraints(
                    false,
                    floorToRocketC.last()
            ),
            kMaxVelocity,
            kMaxAcceleration,
            kMaxVoltage
    )

    override val routine = sequential {
        DriveSubsystem.followTrajectory(t_fallOFfHab,
                Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)).beforeStarting { DriveSubsystem.lowGear = true }
        +parallel {
            +Superstructure.kHatchLow
            +followVisionAssistedTrajectory(
                    t_floorToRocketC,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    3.feet)
        }
    }

}