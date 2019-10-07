package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.frc2.command.SendableSubsystemBase
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerDriveBase
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.launchFrequency
import org.team5940.pantry.lib.FishyRobot

class StandardTrajectoryTrackerCommand(
        private val driveSubsystem: SendableSubsystemBase,
        private val drivetrain: TrajectoryTrackerDriveBase,
        val trajectorySource: Source<Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>>
) : FalconCommand(driveSubsystem) {

    lateinit var job: Job

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        drivetrain.trajectoryTracker.reset(trajectorySource())
        LiveDashboard.isFollowingPath = true
        job = GlobalScope.launchFrequency(frequency = 100) {
            if(!isFinished) {
                drivetrain.setOutput(drivetrain.trajectoryTracker.nextState(drivetrain.robotPosition))
                val referencePoint = drivetrain.trajectoryTracker.referencePoint
                if (referencePoint != null) {
                    val referencePose = referencePoint.state.state.pose

                    // Update Current Path Location on Live Dashboard
                    LiveDashboard.pathX = referencePose.translation.x.feet
                    LiveDashboard.pathY = referencePose.translation.y.feet
                    LiveDashboard.pathHeading = referencePose.rotation.radian
                }
            }
        }
        job.start()
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override fun end(interrupted: Boolean) {
        job.cancel()
        drivetrain.zeroOutputs()
        LiveDashboard.isFollowingPath = false

    }

    override fun isFinished() = drivetrain.trajectoryTracker.isFinished
}
