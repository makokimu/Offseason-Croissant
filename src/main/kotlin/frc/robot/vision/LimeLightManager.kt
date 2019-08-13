package frc.robot.vision

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.experimental.command.SendableSubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.Length
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import kotlin.math.*

// @Suppress("FunctionName")
// fun Translation2d(
//    distance: Double = 0.0,
//    rotation: Rotation2d = Rotation2d()
// ) = Translation2d(distance * rotation.cos, distance * rotation.sin)

object LimeLightManager : SendableSubsystemBase() {

    override fun periodic() = updateFromEstimatedTargetDistance(
            DriveSubsystem.robotPosition,
            Timer.getFPGATimestamp() - pipelineLatency)

    private val table = NetworkTableInstance.getDefault().getTable("limelight")
    private val txEntry = table.getEntry("tx")

    private operator fun NetworkTableEntry.invoke() = getDouble(0.0)

    private fun updateFromEstimatedTargetDistance(robotPosition: Pose2d, timestamp: Double) {

        val distance = getDistanceToTarget()
        val angle = -txEntry().degree

        val estimatedPose: Pose2d? = Pose2d(Translation2d(distance, angle.toRotation2d())).let {

            if (!(it.translation.x.absoluteValue > (Constants.kRobotLength / 2.0 - 5.inch) ||
                    it.translation.y.absoluteValue > (Constants.kRobotWidth / 2.0))) return@let null

            val toReturn = robotPosition + (Constants.kCenterToFrontCamera + it)
//            println("returning ${toReturn.translation.x}, ${toReturn.translation.y}")

            toReturn
        }

        TargetTracker.addSamples(
                timestamp, listOfNotNull(estimatedPose)
        )
    }

    private val pipelineLatency
        get() = (table.getEntry("tl").getDouble(0.0) + 11) / 1000.0

    private fun getDistanceToTarget(): Length {
        val focalLen = 707.0 * (57.0 / 53.0) // = (isHighRes) ? x_focal_length_high : x_focal_length_low;
        val width = 14.6.inch
        val targetSizePx = table.getEntry("tlong").getDouble(0.0) // getTargetXPixels();
        val hypotenuse = width * focalLen / targetSizePx
        val deltaElevation = (45 - 29).inch
        // since a^2 + b^2 = c^2, we find a^2 = c^2 - b^2
        return sqrt(
                hypotenuse.meter.pow(2) - deltaElevation.meter.pow(2)
        ).meter
    }
}