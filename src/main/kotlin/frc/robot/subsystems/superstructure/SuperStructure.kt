package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.PrintCommand
import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import frc.robot.Robot
import frc.robot.subsystems.intake.Intake
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.SIRotationConstants
import org.team5940.pantry.lib.radianToDegree
import java.lang.Math.toDegrees

typealias SuperstructureState = Superstructure.State.Position

object Superstructure : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    val kStowed
            get() = everythingMoveTo(29.inch, (-70).degree, 36.degree)

    val kBackHatchFromLoadingStation = SyncedMove.frontToBack
    val kHatchLow
            get() = everythingMoveTo(16.5.inch, 0.degree, 4.degree)
    val kHatchMid get() = everythingMoveTo(43.inch, 0.degree, 4.degree)
    val kHatchHigh get() = everythingMoveTo(64.5.inch, 0.degree, 4.degree)

    val kCargoIntake get() = everythingMoveTo(24.75.inch, (-44).degree, (-20).degree)
    val kCargoShip get() = everythingMoveTo(47.5.inch, (-5).degree, (-50).degree)
    val kCargoLow get() = everythingMoveTo(20.5.inch, 6.degree, 16.degree)
    val kCargoMid get() = everythingMoveTo(45.5.inch, 6.degree, 16.degree)
    val kCargoHigh get() = everythingMoveTo(65.5.inch, 9.degree, 20.degree)

    fun everythingMoveTo(elevator: Length, proximal: UnboundedRotation, wrist: UnboundedRotation) = everythingMoveTo(State.Position(elevator, proximal, wrist))
    fun everythingMoveTo(elevator: Double, proximal: Double, wrist: Double) = everythingMoveTo(State.Position(elevator, proximal, wrist))
    fun everythingMoveTo(goalState: State.Position): SendableCommandBase = object : FalconCommand(Superstructure, Proximal, Wrist, Elevator) {
        // This whole {} thing is a Supplier<Command> that will return a Command that moves everything safely (hopefully)

        override fun getName() = "move to ${goalState.asString()}"

        fun planPath(): SendableCommandBase {

            println("========================================")

            val currentStateq = currentState

            // returns if we can move everything at the same time or not
            fun safeToMoveSynced(): Boolean {
                val proximalThreshold = -68
                val nowOutsideCrossbar = currentState.proximal.radianToDegree > proximalThreshold
                val willBeOutsideCrossbar = goalState.proximal.radianToDegree > proximalThreshold
                val mightHitElectronics = (goalState.elevator / SILengthConstants.kInchToMeter < 15 && goalState.proximal.radianToDegree > proximalThreshold) || (goalState.elevator / SILengthConstants.kInchToMeter < 20 && goalState.proximal.radianToDegree < proximalThreshold) // TODO check angles?
                val proximalStartSafe = currentState.proximal.radianToDegree > -80
                val proximalEndSafe = goalState.proximal.radianToDegree > -80
                val startHighEnough = currentState.elevator / SILengthConstants.kInchToMeter > 18
                val endHighEnough = goalState.elevator / SILengthConstants.kInchToMeter > 20
                val needsExceptionForCargoGrab = currentState.proximal.radianToDegree > -62 && currentState.elevator / SILengthConstants.kInchToMeter > 25 && goalState.proximal.radianToDegree > -62
                val needsExceptionForStartingDown = (currentState.proximal.radianToDegree > -130.0 && currentState.proximal.radianToDegree < -75.0)
                val safeToMoveSynced = ((nowOutsideCrossbar && willBeOutsideCrossbar && (!mightHitElectronics || needsExceptionForCargoGrab)) ||
                        (proximalStartSafe && proximalEndSafe && startHighEnough && endHighEnough)) && !needsExceptionForStartingDown

                println("need exception for starting down? $needsExceptionForStartingDown")

                 SmartDashboard.putString("passthru data", "nowOutsideCrossbar " + nowOutsideCrossbar + " willBeOutsideCrossbar " + willBeOutsideCrossbar + " might hit electronics? " + mightHitElectronics +
                         " proximalStartSafe " + proximalStartSafe + " proximalEndSafe? " + proximalEndSafe + " startHighEnough " + startHighEnough +
                         " endHighEnough " + endHighEnough)

                println("Safe to move synced? $safeToMoveSynced")
                return safeToMoveSynced
            }

            // returns which joint to move first
            fun shouldMoveElevatorFirst(): Boolean {
                val proximalThreshold = -18.0
//            var currentState = currentst
                val startAboveSafe = goalState.elevator / SILengthConstants.kInchToMeter > 25.0
                var endAboveSafe = currentState.elevator / SILengthConstants.kInchToMeter > 25.0
                val nowOutsideFrame = currentState.proximal.radianToDegree > proximalThreshold
                val willBeOutsideFrame = goalState.proximal.radianToDegree > proximalThreshold
                val shouldMoveElevatorFirst = (nowOutsideFrame && !willBeOutsideFrame && !startAboveSafe) ||
                        (nowOutsideFrame && willBeOutsideFrame) ||
                        ((-35 >= currentState.proximal.radianToDegree && currentState.proximal.radianToDegree >= -90) && (-50 >= goalState.proximal.radianToDegree && goalState.proximal.radianToDegree >= -100))

                println("requested state: $goalState")

                println((if (shouldMoveElevatorFirst) "We are moving the elevator first!" else "We are moving the arm first!"))

                return shouldMoveElevatorFirst
            }

            return sequential {

                // first check if we need to pass through front to back or not
                if (!currentState.isPassedThrough and goalState.isPassedThrough) +SyncedMove.shortPassthrough

                // next check if we can move everything at once
                if (safeToMoveSynced()) {
                    +parallel {
                        +PrintCommand("Moving elevator and prox and wrist")
                        +ClosedLoopElevatorMove(goalState.elevator)
                        +ClosedLoopProximalMove(goalState.proximal)
                        +ClosedLoopWristMove(goalState.wrist)
                        +PrintCommand("Everything moved!")
                    }
                } else {
                    // otherwise, pick the elevator or elbow/wrist to move first
                    if (shouldMoveElevatorFirst()) {
                        +sequential {
                            +PrintCommand("Moving elevator...")
                            +ClosedLoopElevatorMove(goalState.elevator)
                            +PrintCommand("Elevator moved!")
                            +parallel {
                                +PrintCommand("Moving prox and wrist...")
                                +ClosedLoopProximalMove(goalState.proximal)
                                +ClosedLoopWristMove(goalState.wrist)
                                +PrintCommand("Prox and wrist moved")
                            }
                        }
                    } else {
                        // move arm first
                        +sequential {
                            +parallel {
                                +PrintCommand("Moving prox and wrist then elevator")
                                +ClosedLoopProximalMove(goalState.proximal)
                                +ClosedLoopWristMove(goalState.wrist)
                                +PrintCommand("both moved!")
                            }
                            +PrintCommand("Moving elevator")
                            +ClosedLoopElevatorMove(goalState.elevator)
                        }
                    }
                }
                +PrintCommand("[Superstructure Planner] =======> MOVE COMPLETE")
            }
        }

        var path: SendableCommandBase? = null
        var pathStarted = false

        override fun initialize() {
            path = planPath()
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

    fun getUnDumbWrist(dumbWrist: UnboundedRotation, relevantProx: UnboundedRotation) =
            dumbWrist.plus(relevantProx.div(2))

    fun getUnDumbWrist(dumbWrist: Double, relevantProx: Double) =
            dumbWrist.plus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: UnboundedRotation, relevantProx: UnboundedRotation) =
            smartWrist.minus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: Double, relevantProx: Double) =
            smartWrist - (relevantProx / 2)

    override fun activateEmergency() { listOf(Elevator, Proximal, Wrist).forEach { it.activateEmergency() } }

    override fun recoverFromEmergency() { listOf(Elevator, Proximal, Wrist).forEach { it.recoverFromEmergency() } }

    override fun setNeutral() {
        Elevator.wantedState = Elevator.WantedState.Nothing
        Proximal.wantedState = Proximal.WantedState.Nothing
        Wrist.wantedState = Wrist.WantedState.Nothing

        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral()
    }

    var currentState: State.Position = State.Position()
        @Synchronized private set
        @Synchronized get

    override fun periodic() {
        SmartDashboard.putString("Superstructurestate", currentState.asString())
    }

    @Synchronized
    override fun updateState() {
        // update the states of our components
        Elevator.updateState()
        Proximal.updateState()
        Wrist.updateState()

        // use these updated states to build our current state
        // I actually don't think this needs to be syncronised, bc the getter/setter on currentState already are
        synchronized(currentState) { currentState = State.Position(
                Elevator.currentState.position,
                Proximal.currentState.position,
                Wrist.currentState.position,
                wristUnDumb = false
        ) }
    }

    @Synchronized
    override fun useState() {
        Elevator.useState()
        Proximal.useState()
        Wrist.useState()
    }

    override fun lateInit() {
        Elevator.encoder.resetPosition(0.0)
        Proximal.encoder.resetPosition(0.0)
        Wrist.encoder.resetPosition(0.0)

        Robot.subsystemUpdateList.plusAssign(this)

        defaultCommand = JogElevator()

        val zero = ZeroSuperStructureRoutine()
        zero.schedule()
        SmartDashboard.putData(zero)
    }

    sealed class State {

        object Nothing : State()

        data class Position(
            val elevator: Double,
            val proximal: Double,
            val wrist: Double,
            val isPassedThrough: Boolean = proximal < Math.toRadians(-135.0),
            val isWristUnDumb: Boolean = false
        ) : State() {

            @Suppress("unused")
            constructor(
                elevator: Length,
                proximal: UnboundedRotation,
                wrist: UnboundedRotation,
                isWristUnDumb: Boolean = false
            ) :
                    this(elevator.value, proximal.value, (if (isWristUnDumb) getDumbWrist(wrist, proximal) else wrist).value)

            @Suppress("unused")
            internal constructor(
                elevator: Double,
                proximal: Double,
                wrist: Double,
                wristUnDumb: Boolean = false
            ) :
                    this(elevator, proximal, (if (wristUnDumb) getDumbWrist(wrist, proximal) else wrist), isWristUnDumb = wristUnDumb)

            constructor() : this(20.inch, 0.degree, 0.degree) // semi-sane numbers?

            fun proximalTranslation()
                    = Translation2d(kProximalLen.meter, proximal.radian.toRotation2d())

            fun dumbState() = if (!isWristUnDumb) this else Position(elevator, proximal, getDumbWrist(wrist, proximal))
            fun trueState() = if (isWristUnDumb) this else Position(elevator, proximal, getUnDumbWrist(wrist, proximal))

            fun asString(): String {
                return "state: Elevator ${elevator / SILengthConstants.kInchToMeter} proximal ${toDegrees(proximal)} wrist ${toDegrees(wrist)}"
            }
        }

        abstract class CustomState : State() {
            abstract fun useState()
        }
    }
}