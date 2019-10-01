package frc.robot.auto.routines

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
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRadiusConstraint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.utils.withEquals

class YeOldeLowRocketAuto : AutoRoutine() {

    override val duration: SIUnit<Second>
        get() = 0.second



    override val routine = sequential {
        +parallel {
            +Superstructure.everythingMoveTo(19.inch, 0.degree, 4.degree)
            +followVisionAssistedTrajectory(
                    path,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    3.feet)
        }
    }

    companion object {
        private var waypoints = listOf(
                TrajectoryWaypoints.kSideStart.asWaypoint(),
                TrajectoryFactory.rocketNAdjusted
        )

        val path = TrajectoryFactory.generateTrajectory(
                false,
                waypoints,
                listOf(VelocityLimitRadiusConstraint(waypoints.first().position.translation, 4.feet, 3.feet.velocity)),
                kMaxVelocity,
                kMaxAcceleration,
                kMaxVoltage
        )
    }

}