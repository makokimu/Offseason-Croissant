package frc.robot.auto.routines

import edu.wpi.first.wpilibj.experimental.command.*
import frc.robot.Constants
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.VisionAssistedTrajectoryTracker
import org.ghrobotics.lib.commands.parallelRace
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.map

open class AutoRoutine() : SequentialCommandGroup() {

    constructor(wrappedCommand: SendableCommandBase) : this() {
        +wrappedCommand
    }

    // Experimental!!
    fun withExit(exit: BooleanSource): SendableCommandBase {
        return parallelRace {
            +this@AutoRoutine
            +WaitUntilCommand(exit)
        }
    }

    fun followVisionAssistedTrajectory(
        originalTrajectory: TimedTrajectory<Pose2dWithCurvature>,
        pathMirrored: BooleanSource,
        radiusFromEnd: Length,
        useAbsoluteVision: Boolean = false
    ) = VisionAssistedTrajectoryTracker(
            pathMirrored.map(originalTrajectory.mirror(), originalTrajectory),
            radiusFromEnd,
            useAbsoluteVision
    )

    protected fun relocalize(position: Pose2d, forward: Boolean, pathMirrored: BooleanSource) = InstantCommand(Runnable{
        val newPosition = Pose2d(
                pathMirrored.map(position.mirror, position)().translation, // if pathMirrored is true, mirror the pose
                // otherwise, don't. Use that translation2d for the new position
                Drive.localization().rotation
        ) + if (forward) Constants.kForwardIntakeToCenter else Constants.kBackwardIntakeToCenter
        println("RESETTING LOCALIZATION TO ${newPosition.asString()}")
        Drive.localization.reset(newPosition)
    })

    private fun Pose2d.asString() = "Pose X:${translation.x/SILengthConstants.kFeetToMeter}\' Y:${translation.y/SILengthConstants.kFeetToMeter}' Theta:${rotation.degree}deg"

    fun notWithinRegion(region: Rectangle2d) = object : SendableCommandBase() {
        override fun isFinished() = !region.contains(Drive.robotPosition.translation)
    }

    operator fun Command.unaryPlus() {
        addCommands(this@unaryPlus)
    }
}

fun SendableCommandBase.withTimeout(second: Time) = parallelRace {
    +this@withTimeout
    +WaitCommand(second.second)
}