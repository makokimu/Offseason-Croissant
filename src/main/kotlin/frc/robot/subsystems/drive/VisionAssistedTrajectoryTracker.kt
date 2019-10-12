package frc.robot.subsystems.drive

import frc.robot.Constants
import frc.robot.Network
import frc.robot.subsystems.superstructure.Length
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.min
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.acceleration
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.utils.Source

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class VisionAssistedTrajectoryTracker(
    val trajectorySource: Source<Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>>,
    private val radiusFromEnd: Length,
    private val useAbsoluteVision: Boolean = false
) : FalconCommand(DriveSubsystem) {

//    private var visionFinished = false

    private var prevError = 0.0

    @Suppress("LateinitUsage")
    private lateinit var trajectory: Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>

    override fun isFinished() = DriveSubsystem.trajectoryTracker.isFinished // visionFinished

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        trajectory = trajectorySource()
        DriveSubsystem.trajectoryTracker.reset(trajectory)
        LiveDashboard.isFollowingPath = true
//        visionFinished = false
        println("VISION INIT")
    }

    private var lastKnownTargetPose: Pose2d? = null

    override fun execute() {
        val robotPositionWithIntakeOffset = DriveSubsystem.robotPosition // IntakeSubsystem.robotPositionWithIntakeOffset

        val nextState = DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition)

        val withinVisionRadius =
                robotPositionWithIntakeOffset.translation.distance(
                        trajectory.lastState.state.pose.translation // + Translation2d(
//                                Length.kZero,
//                                IntakeSubsystem.badIntakeOffset
//                        )
                ) < radiusFromEnd.value

        // only check for new targets if we are close to the end of the spline
        if (withinVisionRadius) {
            val newTarget = if (!useAbsoluteVision) {
                TargetTracker.getBestTarget(!trajectory.reversed)
            } else {
                TargetTracker.getAbsoluteTarget((trajectory.lastState.state.pose + Constants.kCenterToForwardIntake).translation)
            }

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) this.lastKnownTargetPose = newPose
        }

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose != null) {
            println("VISION")
            visionActive = true
            val transform = lastKnownTargetPose inFrameOfReferenceOf robotPositionWithIntakeOffset
            val angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val error = (angle + if (!trajectory.reversed) Rotation2d() else Math.PI.radian.toRotation2d()).radian
            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)

            // get the distance from the target for the distance P controller
            val distance = transform.translation.norm
            val linear = (distance * kLinearKp).velocity

            // update if our timer is done
//            this.visionFinished = DriveSubsystem.trajectoryTracker.isFinished // inTheEndgame && endgameTimer.hasPeriodPassed(kLinearEndTimeout.second)

            DriveSubsystem.setOutput(
                    TrajectoryTrackerOutput(
                            min(linear, kMaxLinearVelocityVision),
                            0.meter.acceleration,
                            turn.radian.velocity,
                            0.radian.acceleration
                    )
            )

            prevError = error
        } else {
            println("DONT SEE VISION")
            DriveSubsystem.setOutput(nextState)
        }

        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }

//        trajectoryFinished = DriveSubsystem.trajectoryTracker.isFinished
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override fun end(interrupted: Boolean) {
        DriveSubsystem.zeroOutputs()
        LiveDashboard.isFollowingPath = false
        visionActive = false
    }

    companion object {
        const val kCorrectionKp = 0.8 * 1.2 * 1.5//5.5 * 2.0
        const val kCorrectionKd = 8.0//5.0
        const val kLinearKp = 0.6
        val kMaxLinearVelocityVision = 2.5.feet.velocity
        var visionActive = false
    }
}