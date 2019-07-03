/**
 * Some implementation from Team 5190 Green Hope Robotics
 */

package frc.robot.auto

import frc.robot.Network
import frc.robot.auto.paths.TrajectoryWaypoints
//import frc.robot.auto.routines.BottomRocketRoutine
//import frc.robot.auto.routines.CargoShipRoutine
//import frc.robot.auto.routines.HybridRoutine
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.monitor

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition = { Network.startingPositionChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    // Stores if we are ready to send it.
    private val isReady = configValid

    // Update the autonomous listener.
    fun s3ndIt() {
        // Update localization.
        DriveSubsystem.localization.reset(startingPositionMonitor.lastValue.pose)

//        isReadyMonitor.onChangeToTrue {
        JUST S3ND IT
//        }
    }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(startingPosition) {
        state(Autonomous.StartingPositions.LEFT, Autonomous.StartingPositions.RIGHT) {
            stateCommandGroup(autoMode) {
//                state(Autonomous.Mode.TEST_TRAJECTORIES, TestTrajectoriesRoutine())
                state(Autonomous.Mode.FORWARD_CARGO_SHIP, sequential {})
                state(Autonomous.Mode.DO_NOTHING, sequential {})
//                state(Autonomous.Mode.BOTTOM_ROCKET, BottomRocketRoutine())
//                state(Autonomous.Mode.SIDE_CARGO_SHIP, CargoShipRoutine(CargoShipRoutine.Mode.SIDE))
                state(Autonomous.Mode.HYBRID_LEFT, sequential {})
                state(Autonomous.Mode.HYBRID_RIGHT, sequential {})
            }
        }
        state(Autonomous.StartingPositions.CENTER) {
            stateCommandGroup(autoMode) {
//                state(Autonomous.Mode.FORWARD_CARGO_SHIP, CargoShipRoutine(CargoShipRoutine.Mode.FRONT))
//                state(Autonomous.Mode.TEST_TRAJECTORIES, TestTrajectoriesRoutine())
                state(Autonomous.Mode.BOTTOM_ROCKET, sequential {})
                state(Autonomous.Mode.SIDE_CARGO_SHIP, sequential {})
//                state(Autonomous.Mode.HYBRID_LEFT, HybridRoutine(HybridRoutine.Mode.LEFT))
//                state(Autonomous.Mode.HYBRID_RIGHT, HybridRoutine(HybridRoutine.Mode.RIGHT))
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
//    private val modeMonitor = { Robot.lastRobotMode }.monitor


    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart.mirror),
        CENTER(TrajectoryWaypoints.kCenterStart),
        RIGHT(TrajectoryWaypoints.kSideStart)
    }

    enum class Mode { TEST_TRAJECTORIES, BOTTOM_ROCKET, FORWARD_CARGO_SHIP, SIDE_CARGO_SHIP, HYBRID_LEFT, HYBRID_RIGHT, DO_NOTHING }
}