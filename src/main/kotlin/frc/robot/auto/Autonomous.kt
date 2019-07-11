// Implementation from Team 5190 Green Hope Robotics

package frc.robot.auto

import frc.robot.Network
import frc.robot.Robot
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.auto.routines.*
import frc.robot.subsystems.drive.DriveSubsystem
import kotlinx.coroutines.ObsoleteCoroutinesApi
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.team5940.pantry.lib.FishyRobot

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition: () -> StartingPositions = { Network.startingPositionChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    // Stores if we are ready to send it.
    @ObsoleteCoroutinesApi
    private val isReady =
            { Robot.lastRobotMode == FishyRobot.Mode.AUTONOMOUS && Robot.isEnabled } and configValid

    // Update the autonomous listener.
    @ObsoleteCoroutinesApi
    fun update() {
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
//        state(
//                Autonomous.StartingPositions.LEFT,
//                Autonomous.StartingPositions.RIGHT,
//                Autonomous.StartingPositions.LEFT_REVERSED,
//                Autonomous.StartingPositions.RIGHT_REVERSED
//        ) {
//            stateCommandGroup(autoMode) {
//                state(Autonomous.Mode.TEST_TRAJECTORIES, block = TestTrajectoriesRoutine())
//                state(Autonomous.Mode.FORWARD_CARGO_SHIP, sequential {})
//                state(Autonomous.Mode.DO_NOTHING, sequential {})
//                state(Autonomous.Mode.BOTTOM_ROCKET, BottomRocketRoutine()())
//                state(Autonomous.Mode.BOTTOM_ROCKET_2, BottomRocketRoutine2()())
//                state(Autonomous.Mode.SIDE_CARGO_SHIP, CargoShipRoutine(CargoShipRoutine.Mode.SIDE)())
//                state(Autonomous.Mode.HYBRID_LEFT, sequential {})
//                state(Autonomous.Mode.HYBRID_RIGHT, sequential {})
//            }
//        }
//        state(Autonomous.StartingPositions.CENTER) {
//            stateCommandGroup(autoMode) {
//                state(Autonomous.Mode.FORWARD_CARGO_SHIP, CargoShipRoutine(CargoShipRoutine.Mode.FRONT)())
//                state(Autonomous.Mode.TEST_TRAJECTORIES, block = TestTrajectoriesRoutine())
//                state(Autonomous.Mode.BOTTOM_ROCKET, sequential {})
//                state(Autonomous.Mode.BOTTOM_ROCKET_2, sequential {})
//                state(Autonomous.Mode.SIDE_CARGO_SHIP, sequential {})
//                state(Autonomous.Mode.HYBRID_LEFT, block = HybridRoutine(HybridRoutine.Mode.LEFT))
//                state(Autonomous.Mode.HYBRID_RIGHT, block = HybridRoutine(HybridRoutine.Mode.RIGHT))
//            }
//        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    @ObsoleteCoroutinesApi
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.lastRobotMode }.monitor

    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart.mirror),
        CENTER(TrajectoryWaypoints.kCenterStart),
        RIGHT(TrajectoryWaypoints.kSideStart),
        LEFT_REVERSED(TrajectoryWaypoints.kSideStartReversed.mirror),
        RIGHT_REVERSED(TrajectoryWaypoints.kSideStartReversed)
    }

    enum class Mode { TEST_TRAJECTORIES, BOTTOM_ROCKET, BOTTOM_ROCKET_2, FORWARD_CARGO_SHIP, SIDE_CARGO_SHIP, HYBRID_LEFT, HYBRID_RIGHT, DO_NOTHING }
}