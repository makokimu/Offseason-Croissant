package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second

class DriveTrajectory(
    val trajectory: Trajectory<Time, TimedEntry<Pose2dWithCurvature>>,
    val tracker: TrajectoryTracker,
    val reset: Boolean
) : FalconCommand(DriveSubsystem) {

    override fun initialize() {
        tracker.reset(trajectory)

        LiveDashboard.isFollowingPath = false
        LiveDashboard.isFollowingPath = true

        if (reset) {
            DriveSubsystem.localization.reset(trajectory.firstState.state.pose)
        }
    }

    override fun execute() {
        val nextState = tracker.nextState(DriveSubsystem.robotPosition,
                Timer.getFPGATimestamp().second)

        val refState: Pose2d? = tracker.referencePoint?.state?.state?.pose

        if (refState != null) {
            LiveDashboard.pathX = refState.translation.x / SILengthConstants.kFeetToMeter
            LiveDashboard.pathY = refState.translation.y / SILengthConstants.kFeetToMeter
            LiveDashboard.pathHeading = refState.rotation.radian
        }

        DriveSubsystem.setOutput(nextState)
    }
}