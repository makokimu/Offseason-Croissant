// Implementation from Team 5190 Green Hope Robotics

package frc.robot.auto

import frc.robot.Network
import frc.robot.Robot
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.auto.routines.BottomRocketRoutine2
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.*
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
            startingPosition.withEquals(Autonomous.StartingPositions.LEFT) or
                    startingPosition.withEquals(Autonomous.StartingPositions.LEFT_REVERSED)

    // Stores if we are ready to send it.
    private val isReady =
            { Robot.isAuto && Robot.isEnabled && configValid }

    @Suppress("LocalVariableName")
    private val IT = ""

    // Update the autonomous listener.
    override fun update() {
        // Update localization.
        startingPositionMonitor.onChange { if (!Robot.isEnabled) DriveSubsystem.localization.reset(it.pose) }

//        modeMonitor.onChange { newValue ->
//            if (newValue != FishyRobot.Mode.AUTONOMOUS) JUST.end(true)
//        }

//        isReadyMonitor.onChangeToTrue {
//            JUST S3ND IT
//        }
    }

    private val masterGroup = hashMapOf(
            StartingPositions.CENTER to hashMapOf(
                    Mode.DO_NOTHING to sequential { }
            ),
            StartingPositions.LEFT to hashMapOf(
                    Mode.DO_NOTHING to sequential { }
            ),
            StartingPositions.RIGHT to hashMapOf(
                    Mode.DO_NOTHING to sequential { }
            ),
            StartingPositions.LEFT_REVERSED to hashMapOf(
                    Mode.DO_NOTHING to sequential { },
                    Mode.BOTTOMROCKETREVERSED to BottomRocketRoutine2()()
            ),
            StartingPositions.RIGHT_REVERSED to hashMapOf(
                    Mode.DO_NOTHING to sequential { },
                    Mode.BOTTOMROCKETREVERSED to BottomRocketRoutine2()()
            )
    )

    private val configValid = masterGroup[startingPosition()] == null && masterGroup[startingPosition()]?.get(autoMode()) != null

//    private val JUST: CommandGroupBase
//        get() = masterGroup[startingPosition()]?.get(autoMode()) ?: sequential { }

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
    enum class Mode {
        YEOLDEROUTINE,
        DO_NOTHING,
        BOTTOMROCKETREVERSED
    }
}