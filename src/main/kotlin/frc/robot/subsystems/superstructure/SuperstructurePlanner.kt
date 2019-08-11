package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import edu.wpi.first.wpilibj.experimental.command.WaitUntilCommand
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import frc.robot.auto.routines.withExit
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.min
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.team5940.pantry.lib.SIRotationConstants
import org.team5940.pantry.lib.degreeToRadian
import java.lang.IllegalStateException
import kotlin.math.min

object SuperstructurePlanner {

    private val kCrossbar = 33.inch
    private val kMinimumSafeSyncedProximal = (-75).degree
    private val kOutsideFrame = (-50).degree
//    val kProximalLen = 32.inch.meter

    private fun planPath(currentState: SuperstructureState, goalState: SuperstructureState) = sequential {

        // first check passthrough -- remember, if we do pass through, our "current state" will change!
        val needsPassthrough = !currentState.isPassedThrough and goalState.isPassedThrough

        if (needsPassthrough) {
            val worstCaseProximalAfterPassthrough =
                    worstCaseProximalTipElevation(SuperstructureState(kCrossbar, kMinimumSafeSyncedProximal, 0.degree), goalState)

            if (worstCaseProximalAfterPassthrough > 2.inch && goalState.proximal > kMinimumSafeSyncedProximal) {
                // synced safe, as it's an upward(ish, technically) move that's outside the crossbar
                println("                // synced safe, as it's an upward(ish, technically) move that's outside the crossbar")
                +ClosedLoopElevatorMove(kCrossbar)
                +parallel {
                    +SyncedMove(0.0.degree, false)
                    +sequential {
                        +WaitUntilCommand { Superstructure.currentState.proximal > (-75).degree }
                        +ClosedLoopElevatorMove(goalState.elevator)
                    }
                } // these have to be outside the parallel group because SyncedMove reserves the proximal and wrist
                +ClosedLoopWristMove(goalState.wrist)
                +ClosedLoopProximalMove(goalState.proximal)
            } else if (worstCaseProximalAfterPassthrough < 2.inch &&
                    goalState.proximal > kOutsideFrame) {
                // arm needs to move all the way out before we start moving the elevator
                println("                // arm needs to move all the way out before we start moving the elevator")
                +ClosedLoopElevatorMove(kCrossbar)
                +SyncedMove(0.0.degree, false)
                +parallel {
                    +ClosedLoopElevatorMove(goalState.elevator)
                    +ClosedLoopProximalMove(goalState.proximal)
                    +ClosedLoopWristMove(goalState.wrist)
                }
            } else if (goalState.elevator < kCrossbar &&
                    goalState.proximal < kMinimumSafeSyncedProximal &&
                    goalState.proximalTranslation().y > 2.inch) {
                // below the crossbar and below -75 degrees
                println("// below the crossbar and below -75 degrees")
                +ClosedLoopElevatorMove(kCrossbar)
                +SyncedMove(0.0.degree, false)
                +parallel {
                    +ClosedLoopProximalMove(goalState.proximal)
                    +ClosedLoopWristMove(goalState.wrist)
                }
                +ClosedLoopElevatorMove(goalState.elevator)
            } else {
                throw IllegalStateException("Move is impossible, maybe!")
            }
        } else {
            // doesn't need passthrough, go through the checks again
            val worstCaseProximal =
                    worstCaseProximalTipElevation(currentState, goalState)

            if ((worstCaseProximal > 2.inch ||
                            min(currentState.proximal, goalState.proximal) > kOutsideFrame) && /* exception for cargo grab */
                    goalState.proximal > kMinimumSafeSyncedProximal) {
                // synced safe, as it's an upward(ish, technically) move that's outside the crossbar
                println("// synced safe, as it's an upward(ish, technically) move that's outside the crossbar")
                +parallel {
                    +ClosedLoopElevatorMove(goalState.elevator)
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(goalState.proximal)
                }
            }
            // next arm first vs elevator first
            else if (goalState.proximalTranslation().y < 2.inch ||
                    min(currentState.proximal, goalState.proximal) < kMinimumSafeSyncedProximal &&
                    goalState.proximal > kOutsideFrame) {
                // arm first if it's a downward move to below electronics or if we're inside the crossbar
                println("// arm first if it's a downward move to below electronics or if we're inside the crossbar")
                +parallel {
                    val armMove = parallel { +ClosedLoopWristMove(goalState.wrist)
                        +ClosedLoopProximalMove(goalState.proximal) }
                    +armMove
                    +sequential {
                        +WaitUntilCommand { Superstructure.currentState.proximal > kOutsideFrame }.withExit { armMove.isFinished }
                        +ClosedLoopElevatorMove(goalState.elevator)
                    }
                }
            } else if (currentState.elevator > goalState.elevator &&
                    goalState.elevator < kCrossbar &&
                    currentState.proximal > kMinimumSafeSyncedProximal &&
                    goalState.proximal < kMinimumSafeSyncedProximal
                    ) {
                // weird handler for downward moves to somewhere within the crossbar
                println("// weird handler for downward moves to somewhere within the crossbar")

                // move the elevator down then move joints
                +parallel {
                    val elevatorMove = ClosedLoopElevatorMove(goalState.elevator)
                    +elevatorMove
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(min(goalState.proximal, kMinimumSafeSyncedProximal))
                    +sequential {
                        +WaitUntilCommand { Superstructure.currentState.elevator < kCrossbar }.withExit { elevatorMove.isFinished }
                        +ClosedLoopWristMove(goalState.wrist)
                        +ClosedLoopProximalMove(goalState.proximal)
                    }
                }
            } else if (worstCaseProximal < 2.inch &&
                    currentState.proximal > kOutsideFrame &&
                    goalState.proximal < kOutsideFrame) {
                // elevator first (i.e. coming from cargo grab to stowed)
                +ClosedLoopElevatorMove(goalState.elevator)
                +parallel {
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(goalState.proximal)
                }
            } else {
                throw IllegalStateException("Cannot find a safe path from ${currentState.asString()} to ${goalState.asString()}")
            }
        }
    }

    private fun worstCaseProximalTipElevation(currentState: SuperstructureState, goalState: SuperstructureState): Length {
        val worstArmTranslation = Translation2d(kProximalLen, min(currentState.proximal, goalState.proximal).minus(5.degree).toRotation2d())
        return min(currentState.elevator, goalState.elevator) + worstArmTranslation.y
    }

    fun everythingMoveTo(goalState: SuperstructureState): SendableCommandBase =
            object : FalconCommand(Superstructure, Proximal, Wrist, Elevator) {
                // This whole {} thing is a Supplier<Command> that will return a Command that moves everything safely (hopefully)

                override fun getName() = "move to ${goalState.asString()}"

                var path: SendableCommandBase? = null
                var pathStarted = false

                override fun initialize() {
                    path = planPath(Superstructure.currentState, goalState)
                    path!!.initialize()
                    pathStarted = true
                }

                override fun execute() {
                    path!!.execute()
                }

                override fun end(interrupted: Boolean) {
                    path?.end(interrupted)
                    path = null
                    pathStarted = false
                }

                override fun isFinished(): Boolean {
                    val path = this.path
                    return path?.isFinished ?: pathStarted // if the path is null, check that it's started, otherwise call the path's isFinished() method
                }
            }
}

//    fun everythingMoveTo(goalState: State.Position): SendableCommandBase = object : FalconCommand(Superstructure, Proximal, Wrist, Elevator) {
//        // This whole {} thing is a Supplier<Command> that will return a Command that moves everything safely (hopefully)
//
//        override fun getName() = "move to ${goalState.asString()}"
//
//        fun planPath(): SendableCommandBase {
//
//            println("========================================")
//
//            val currentStateq = currentState
//
//            // returns if we can move everything at the same time or not
//            fun safeToMoveSynced(): Boolean {
//                val proximalThreshold = -68
//                val nowOutsideCrossbar = currentState.proximal.radianToDegree > proximalThreshold
//                val willBeOutsideCrossbar = goalState.proximal.radianToDegree > proximalThreshold
//                val mightHitElectronics = (goalState.elevator / SILengthConstants.kInchToMeter < 15 && goalState.proximal.radianToDegree > proximalThreshold) || (goalState.elevator / SILengthConstants.kInchToMeter < 20 && goalState.proximal.radianToDegree < proximalThreshold) // TODO check angles?
//                val proximalStartSafe = currentState.proximal.radianToDegree > -80
//                val proximalEndSafe = goalState.proximal.radianToDegree > -80
//                val startHighEnough = currentState.elevator / SILengthConstants.kInchToMeter > 18
//                val endHighEnough = goalState.elevator / SILengthConstants.kInchToMeter > 20
//                val needsExceptionForCargoGrab = currentState.proximal.radianToDegree > -62 && currentState.elevator / SILengthConstants.kInchToMeter > 25 && goalState.proximal.radianToDegree > -62
//                val needsExceptionForStartingDown = (currentState.proximal.radianToDegree > -130.0 && currentState.proximal.radianToDegree < -75.0)
//                val safeToMoveSynced = ((nowOutsideCrossbar && willBeOutsideCrossbar && (!mightHitElectronics || needsExceptionForCargoGrab)) ||
//                        (proximalStartSafe && proximalEndSafe && startHighEnough && endHighEnough)) && !needsExceptionForStartingDown
//
//                println("need exception for starting down? $needsExceptionForStartingDown")
//
//                 SmartDashboard.putString("passthru data", "nowOutsideCrossbar " + nowOutsideCrossbar + " willBeOutsideCrossbar " + willBeOutsideCrossbar + " might hit electronics? " + mightHitElectronics +
//                         " proximalStartSafe " + proximalStartSafe + " proximalEndSafe? " + proximalEndSafe + " startHighEnough " + startHighEnough +
//                         " endHighEnough " + endHighEnough)
//
//                println("Safe to move synced? $safeToMoveSynced")
//                return safeToMoveSynced
//            }
//
//            // returns which joint to move first
//            fun shouldMoveElevatorFirst(): Boolean {
//                val proximalThreshold = -18.0
// //            var currentState = currentst
//                val startAboveSafe = goalState.elevator / SILengthConstants.kInchToMeter > 25.0
//                var endAboveSafe = currentState.elevator / SILengthConstants.kInchToMeter > 25.0
//                val nowOutsideFrame = currentState.proximal.radianToDegree > proximalThreshold
//                val willBeOutsideFrame = goalState.proximal.radianToDegree > proximalThreshold
//                val shouldMoveElevatorFirst = (nowOutsideFrame && !willBeOutsideFrame && !startAboveSafe) ||
//                        (nowOutsideFrame && willBeOutsideFrame) ||
//                        ((-35 >= currentState.proximal.radianToDegree && currentState.proximal.radianToDegree >= -90) && (-50 >= goalState.proximal.radianToDegree && goalState.proximal.radianToDegree >= -100))
//
//                println("requested state: $goalState")
//
//                println((if (shouldMoveElevatorFirst) "We are moving the elevator first!" else "We are moving the arm first!"))
//
//                return shouldMoveElevatorFirst
//            }
//
//            return sequential {
//
//                // first check if we need to pass through front to back or not
//                if (!currentState.isPassedThrough and goalState.isPassedThrough) +SyncedMove.shortPassthrough
//
//                // next check if we can move everything at once
//                if (safeToMoveSynced()) {
//                    +parallel {
//                        +PrintCommand("Moving elevator and prox and wrist")
//                        +ClosedLoopElevatorMove(goalState.elevator)
//                        +ClosedLoopProximalMove(goalState.proximal)
//                        +ClosedLoopWristMove(goalState.wrist)
//                        +PrintCommand("Everything moved!")
//                    }
//                } else {
//                    // otherwise, pick the elevator or elbow/wrist to move first
//                    if (shouldMoveElevatorFirst()) {
//                        +sequential {
//                            +PrintCommand("Moving elevator...")
//                            +ClosedLoopElevatorMove(goalState.elevator)
//                            +PrintCommand("Elevator moved!")
//                            +parallel {
//                                +PrintCommand("Moving prox and wrist...")
//                                +ClosedLoopProximalMove(goalState.proximal)
//                                +ClosedLoopWristMove(goalState.wrist)
//                                +PrintCommand("Prox and wrist moved")
//                            }
//                        }
//                    } else {
//                        // move arm first
//                        +sequential {
//                            +parallel {
//                                +PrintCommand("Moving prox and wrist then elevator")
//                                +ClosedLoopProximalMove(goalState.proximal)
//                                +ClosedLoopWristMove(goalState.wrist)
//                                +PrintCommand("both moved!")
//                            }
//                            +PrintCommand("Moving elevator")
//                            +ClosedLoopElevatorMove(goalState.elevator)
//                        }
//                    }
//                }
//                +PrintCommand("[Superstructure Planner] =======> MOVE COMPLETE")
//            }
//        }
//
//        var path: SendableCommandBase? = null
//        var pathStarted = false
//
//        override fun initialize() {
//            path = planPath()
//            path!!.initialize()
//            pathStarted = true
//        }
//
//        override fun execute() {
//            path!!.execute()
//        }
//
//        override fun end(interrupted: Boolean) {
//            path?.end(interrupted)
//            path = null
//            pathStarted = false
//        }
//
//        override fun isFinished(): Boolean {
//            val path = this.path
//            return path?.isFinished ?: pathStarted // if the path is null, check that it's started, otherwise call the path's isFinished() method
//        }
//    }