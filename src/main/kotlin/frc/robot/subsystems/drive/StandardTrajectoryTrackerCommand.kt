package frc.robot.subsystems.drive

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.loopFrequency

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param driveSubsystem Instance of the drive subsystem to use
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class StandardTrajectoryTrackerCommand(
    val trajectorySource: Source<Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>>
) : FalconCommand(DriveSubsystem) {

//    lateinit var notifier: Job

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        DriveSubsystem.trajectoryTracker.reset(trajectorySource())
        LiveDashboard.isFollowingPath = true
//        notifier = GlobalScope.launch {
//            loopFrequency(100) {
//                DriveSubsystem.setOutput(DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition))
//                val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
//                if (referencePoint != null) {
//                    val referencePose = referencePoint.state.state.pose
//
//                    // Update Current Path Location on Live Dashboard
//                    LiveDashboard.pathX = referencePose.translation.x.feet
//                    LiveDashboard.pathY = referencePose.translation.y.feet
//                    LiveDashboard.pathHeading = referencePose.rotation.radian
//                }
//            }
//        }
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
//        notifier.cancel()
        DriveSubsystem.zeroOutputs()
        LiveDashboard.isFollowingPath = false
    }

    override fun isFinished() = DriveSubsystem.trajectoryTracker.isFinished
}
