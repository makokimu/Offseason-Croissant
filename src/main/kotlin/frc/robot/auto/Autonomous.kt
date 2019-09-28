// Implementation from Team 5190 Green Hope Robotics

package frc.robot.auto

import frc.robot.Network
import frc.robot.Robot
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.auto.routines.* // ktlint-disable no-wildcard-imports
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.team5940.pantry.lib.FishyRobot
import org.team5940.pantry.lib.Updatable

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous : Updatable {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition: Source<StartingPositions> = { Network.startingPositionChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = {
        true
    }

    // Stores if we are ready to send it.
    private val isReady =
            { Robot.lastRobotMode == FishyRobot.Mode.AUTONOMOUS && Robot.isEnabled } and configValid

    // Update the autonomous listener.
    override fun update() {
        // Update localization if the startingPositionMonitor value's changed since the last call
        startingPositionMonitor.onChange { if (!Robot.isEnabled) DriveSubsystem.localization.reset(it.pose) }

        // make sure that we're in auto mode (and cancel auto if we aren't)
        modeMonitor.onChange { newValue ->
            if (newValue != FishyRobot.Mode.AUTONOMOUS) JUST.cancel()
        }

        isReadyMonitor.onChangeToTrue {
            JUST S3ND IT
        }
    }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(startingPosition) {
        state(
                StartingPositions.LEFT,
                StartingPositions.RIGHT,
                StartingPositions.LEFT_REVERSED,
                StartingPositions.RIGHT_REVERSED
        ) {
            stateCommandGroup(autoMode) {
                state(Mode.DO_NOTHING, sequential {})
//                state(Mode.YEOLDEROUTINE, YeOldeLowRocketAuto()())
            }
        }
        state(StartingPositions.CENTER) {
            stateCommandGroup(autoMode) {
                state(Mode.DO_NOTHING, sequential {})
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.lastRobotMode }.monitor

    @Suppress("unused")
    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart.mirror),
        CENTER(TrajectoryWaypoints.kCenterStart),
        RIGHT(TrajectoryWaypoints.kSideStart),
        LEFT_REVERSED(TrajectoryWaypoints.kSideStartReversed.mirror),
        RIGHT_REVERSED(TrajectoryWaypoints.kSideStartReversed)
    }

    @Suppress("unused")
    enum class Mode { YEOLDEROUTINE, DO_NOTHING }
}