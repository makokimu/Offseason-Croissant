// Implementation from Team 5190 Green Hope Robotics

package frc.robot.auto

import edu.wpi.first.wpilibj.frc2.command.Command
import edu.wpi.first.wpilibj.frc2.command.CommandGroupBase
import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import edu.wpi.first.wpilibj.frc2.command.SendableCommandBase
import frc.robot.Network
import frc.robot.Robot
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.auto.routines.BottomRocketRoutine2
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import org.team5940.pantry.lib.Updatable

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous : Updatable {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition: Source<StartingPositions> = { Network.startingPositionChooser.selected }

    val isStartingOnLeft: Source<Boolean> =
            startingPosition.withEquals(StartingPositions.LEFT) or
                    startingPosition.withEquals(StartingPositions.LEFT_REVERSED)

    // Stores if we are ready to send it.
    private val isReady =
            { Robot.isAuto && Robot.isEnabled && configValid }

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.currentMode }.monitor


    @Suppress("LocalVariableName")
    private val IT = ""

    // Update the autonomous listener.
    override fun update() {
        // Update localization.
        startingPositionMonitor.onChange { if (!Robot.isEnabled) DriveSubsystem.localization.reset(it.pose) }

        modeMonitor.onChange { newValue ->
            if (newValue != FalconTimedRobot.Mode.AUTONOMOUS) JUST.end(true)
        }

        isReadyMonitor.onChangeToTrue {
            JUST S3ND IT
        }
    }

    fun s3nd() = JUST S3ND IT

    private val masterGroup = hashMapOf(
            StartingPositions.CENTER to hashMapOf(
                    Mode.DO_NOTHING to InstantCommand()
            ),
            StartingPositions.LEFT to hashMapOf(
                    Mode.DO_NOTHING to InstantCommand()
            ),
            StartingPositions.RIGHT to hashMapOf(
                    Mode.DO_NOTHING to InstantCommand()
            ),
            StartingPositions.LEFT_REVERSED to hashMapOf(
                    Mode.DO_NOTHING to InstantCommand(),
                    Mode.BOTTOMROCKETREVERSED to BottomRocketRoutine2()()
            ),
            StartingPositions.RIGHT_REVERSED to hashMapOf(
                    Mode.DO_NOTHING to InstantCommand(),
                    Mode.BOTTOMROCKETREVERSED to BottomRocketRoutine2()()
            )
    )

    private val configValid = masterGroup[startingPosition()] == null && masterGroup[startingPosition()]?.get(autoMode()) != null

    private val JUST: SendableCommandBase
        get() = masterGroup[startingPosition()]?.get(autoMode()) ?: sequential { }

    @Suppress("unused")
    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart.mirror),
        CENTER(TrajectoryWaypoints.kCenterStart),
        RIGHT(TrajectoryWaypoints.kSideStart),
        LEFT_REVERSED(TrajectoryWaypoints.kSideStartReversed.mirror),
        RIGHT_REVERSED(TrajectoryWaypoints.kSideStartReversed)
    }

    @Suppress("unused")
    enum class Mode {
        YEOLDEROUTINE,
        DO_NOTHING,
        BOTTOMROCKETREVERSED
    }
}

@Suppress("UNUSED_PARAMETER")
infix fun Command.S3ND(other: Any) = this.schedule()