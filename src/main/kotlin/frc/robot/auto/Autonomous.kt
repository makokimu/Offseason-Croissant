package frc.robot.auto

import frc.robot.Network
import frc.robot.Robot
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.auto.routines.BottomRocketRoutine2
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import org.team5940.pantry.lib.Updatable

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous : Updatable {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition = { Network.startingPositionChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    val isStartingOnLeft = { val position = startingPosition()
        position == StartingPositions.LEFT ||
            position == StartingPositions.LEFT_REVERSED
    }

    // Stores if we are ready to send it.
    private val isReady =
            { Robot.currentMode == FalconTimedRobot.Mode.AUTONOMOUS && Robot.isEnabled } and configValid

    // Update the autonomous listener.
    override fun update() {
        // Update localization.
        startingPositionMonitor.onChange { if (!Robot.isEnabled) DriveSubsystem.localization.reset(it.pose) }

        modeMonitor.onChange { newValue ->
            if (newValue != FalconTimedRobot.Mode.AUTONOMOUS) JUST.cancel()
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
                state(Mode.TEST_TRAJECTORIES, sequential {}) // TestTrajectoriesRoutine())
                state(Mode.FORWARD_CARGO_SHIP, sequential {})
                state(Mode.DO_NOTHING, sequential {})
                state(Mode.BOTTOM_ROCKET, sequential {}) // BottomRocketRoutine()())
                state(Mode.BOTTOM_ROCKET_2, BottomRocketRoutine2()())
                state(Mode.SIDE_CARGO_SHIP, sequential {}) // CargoShipRoutine(CargoShipRoutine.Mode.SIDE)())
                state(Mode.HYBRID_LEFT, sequential {})
                state(Mode.HYBRID_RIGHT, sequential {})
            }
        }
        state(StartingPositions.CENTER) {
            stateCommandGroup(autoMode) {
                state(Mode.TEST_TRAJECTORIES, sequential {}) // TestTrajectoriesRoutine())
                state(Mode.FORWARD_CARGO_SHIP, sequential {}) // CargoShipRoutine(CargoShipRoutine.Mode.FRONT)())
                state(Mode.DO_NOTHING, sequential {})
                state(Mode.BOTTOM_ROCKET, sequential {})
                state(Mode.BOTTOM_ROCKET_2, sequential {})
                state(Mode.SIDE_CARGO_SHIP, sequential {})
                state(Mode.HYBRID_LEFT, sequential {}) // HybridRoutine(HybridRoutine.Mode.LEFT))
                state(Mode.HYBRID_RIGHT, sequential {}) // HybridRoutine(HybridRoutine.Mode.RIGHT))
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.currentMode }.monitor

    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart.mirror),
        CENTER(TrajectoryWaypoints.kCenterStart),
        RIGHT(TrajectoryWaypoints.kSideStart),
        LEFT_REVERSED(TrajectoryWaypoints.kSideStartReversed.mirror),
        RIGHT_REVERSED(TrajectoryWaypoints.kSideStartReversed)
    }

    enum class Mode { TEST_TRAJECTORIES, BOTTOM_ROCKET, BOTTOM_ROCKET_2, FORWARD_CARGO_SHIP, SIDE_CARGO_SHIP, HYBRID_LEFT, HYBRID_RIGHT, DO_NOTHING }
}