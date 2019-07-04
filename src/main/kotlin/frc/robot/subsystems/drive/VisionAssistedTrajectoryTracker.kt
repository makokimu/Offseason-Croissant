package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import frc.robot.Constants
import frc.robot.Network
import frc.robot.vision.LimeLight
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.utils.Source
import kotlin.math.PI

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class VisionAssistedTrajectoryTracker(
    val trajectorySource: Source<Trajectory<Time, TimedEntry<Pose2dWithCurvature>>>,
    val radiusFromEnd: Length,
    val useAbsoluteVision: Boolean = false,
    val useLimeLightOverTargetTracker: Boolean = true
) : SendableCommandBase() {

    private var trajectoryFinished = false

    var limelightHasTarget = false
    var limeLightAngle: Rotation2d? = null

    private var prevError = 0.0 // cached error for the PD loop

    @Suppress("LateinitUsage")
    private lateinit var trajectory: Trajectory<Time, TimedEntry<Pose2dWithCurvature>>

    override fun isFinished() = trajectoryFinished
    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        trajectory = trajectorySource()
        DriveSubsystem.trajectoryTracker.reset(trajectory)
        trajectoryFinished = false
        LiveDashboard.isFollowingPath = true
    }

    private var lastKnownTargetPose: Pose2d? = null // cache of the vision target pose

    override fun execute() {
//        val robotPositionWithIntakeOffset = IntakeSubsystem.robotPositionWithIntakeOffset

        val robotPosition = DriveSubsystem.robotPosition

        // get the next state from the trajectory tracker
        val nextState = DriveSubsystem.trajectoryTracker.nextState(
                robotPosition)

        // check if we are close enough to the last pose to engage vision
        val withinVisionRadius =
                robotPosition.translation.distance(
                        trajectory.lastState.state.pose.translation
                ) < radiusFromEnd.value

        // if we are, try to find a new target
        if (withinVisionRadius) {

            if (!useLimeLightOverTargetTracker && trajectory.reversed) {

                val newTarget = if (useAbsoluteVision) {
                    TargetTracker.getAbsoluteTarget((trajectory.lastState.state.pose + Constants.kCenterToForwardIntake).translation)
                } else {
                    TargetTracker.getBestTarget(!trajectory.reversed) // usually this is the case
                }

                val newPose = newTarget?.averagedPose2d

                // store this pose if it's for real
                if (newTarget?.isAlive == true && newPose != null) this.lastKnownTargetPose = newPose
            } else if (useLimeLightOverTargetTracker) {

                // we COULD use the limelight
                this.limelightHasTarget = LimeLight.trackedTargets > 0

                this.limeLightAngle = if (limelightHasTarget) {
                    LimeLight.dx + robotPosition.rotation } else null
            }
        }

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose != null || limeLightAngle != null) {
            println("VISION ACTIVE")
            visionActive = true

            var error: Double
            var turn: Double
//            var youShouldUseLimeLight = false

            if (limeLightAngle != null && useLimeLightOverTargetTracker) {

                error = -(limeLightAngle!! - robotPosition.rotation).radian

//                youShouldUseLimeLight = true

                Network.visionDriveAngle.setDouble(Math.toDegrees(error))
                Network.visionDriveActive.setBoolean(true)

                println("limelight angle error: $error")

                turn = kLimeLightKp * error + kLimeLightKd * (error - prevError)
            } else if (lastKnownTargetPose != null) {

                // find our angle to the target
                val transform = lastKnownTargetPose inFrameOfReferenceOf robotPosition
                val angle = Rotation2d(transform.translation.x, transform.translation.y, true)

                Network.visionDriveAngle.setDouble(angle.degree)
                Network.visionDriveActive.setBoolean(true)

                error = (angle + if (!trajectory.reversed) Rotation2d() else Math.PI.radian.toRotation2d()).radian

                println("jevois angle error: $error")

                turn = kJevoisKp * error + kJevoisKd * (error - prevError)
            } else {
                println("NO TARGET FOUND, mega prank (you should never see this...), returning...")
                DriveSubsystem.setOutput(nextState) // go back to RAMSETE tracking mode
                return
            }

            val newCommandedOutput = TrajectoryTrackerOutput(
                    nextState.linearVelocity,
                    0.meter.acceleration,
                    turn.radian.velocity,
                    0.radian.acceleration
            )

            println("demanding output with turn ${turn.radian.velocity.value}")

            // set the drivetrain to the RAMSETE/whatever linear velocity and the PD loop's output for turn
            DriveSubsystem.setOutput(
                    newCommandedOutput
            )

            prevError = error // save error for the PD loop
        } else { // just do the boring Ramsete stuff
            DriveSubsystem.setOutput(nextState)
        }

        // update LiveDashboard
        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x / SILengthConstants.kFeetToMeter
            LiveDashboard.pathY = referencePose.translation.y / SILengthConstants.kFeetToMeter
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }

        trajectoryFinished = DriveSubsystem.trajectoryTracker.isFinished
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
        const val kJevoisKp = 5.5 * (2 * PI) / 360.0
        const val kJevoisKd = 0.0

        const val kLimeLightKp = 0.2 * 3.5
        const val kLimeLightKd = 0.toDouble()

        var visionActive = false
    }
}