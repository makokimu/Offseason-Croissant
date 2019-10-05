package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.frc2.command.SendableSubsystemBase
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerDriveBase
import org.ghrobotics.lib.utils.Source

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param driveSubsystem Instance of the drive subsystem to use
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class StandardTrajectoryTrackerCommand(
        private val driveSubsystem: SendableSubsystemBase,
        private val drivetrain: TrajectoryTrackerDriveBase,
        val trajectorySource: Source<Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>>
) : FalconCommand(DriveSubsystem) {

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        DriveSubsystem.trajectoryTracker.reset(trajectorySource())
        LiveDashboard.isFollowingPath = true
    }

    override fun execute() {
        DriveSubsystem.setOutput(DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition))
        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override fun end(interrupted: Boolean) {
        DriveSubsystem.zeroOutputs()
        LiveDashboard.isFollowingPath = false
    }

    override fun isFinished() = DriveSubsystem.trajectoryTracker.isFinished
}
