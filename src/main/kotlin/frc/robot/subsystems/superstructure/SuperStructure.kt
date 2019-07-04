package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.SelectCommand
import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Robot
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import java.lang.Math.toDegrees

object SuperStructure : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    // preset semi-singleton commands for each superstructure preset
    val kHatchFrontFromLoadingStation: SendableCommandBase = everythingMoveTo(State.Position(19.inch, 0.degree, 0.degree, isWristUnDumb = true))

    fun everythingMoveTo(state: State.Position) = SelectCommand {

        when {
            state.isPassedThrough && !currentState.isPassedThrough -> InstantCommand()
            else -> InstantCommand()
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
        private set
    var wantedState: State = State.Nothing
        private set

    override fun periodic() {
        SmartDashboard.putString("Superstructurestate", currentState.asString())
    }

    // this Runnable will be run to update the State through a separate Thread periodically
    override fun updateState() {
        // update the states of our components
        Elevator.updateState()
        Proximal.updateState()
        Wrist.updateState()

        // use these updated states to build our current state
        synchronized(currentState) {currentState = State.Position(
                Elevator.currentState.position,
                Proximal.currentState.position,
                Wrist.currentState.position,
                wristUnDumb = false
        )}
    }

    fun customizeWantedState(state: State): State {

        // todo modify state (ex vision being blocked, illegal state, etc
        return state
    }

//    override fun useState() {
//        val wantedState = customizeWantedState(this.wantedState)
//
//        if(wantedState !is State.Position) {setNeutral(); return}
//
//        println("moving superstructure to elevator ${wantedState.elevator/SILengthConstants.kInchToMeter}" +
//                "proximal ${wantedState.proximal/(2*Math.PI)*360.0}" +
//                "wrist ${wantedState.wrist/(2*Math.PI)*360.0}")
//
// //        Elevator.wantedState = Elevator.WantedState.Position(wantedState.elevator)
// //        Elevator.useState()
//
//        Proximal.wantedState = Proximal.WantedState.Position(wantedState.proximal)
//        Proximal.useState()
//
// //        Wrist.wantedState = Wrist.WantedState.Position(wantedState.wrist)
// //        Wrist.useState()
//    }

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

        val zero = ZeroSuperStructureRoutine()
        zero.schedule()
        SmartDashboard.putData(zero)

        SmartDashboard.putData(ClosedLoopElevatorMove(25.inch.meter))
        SmartDashboard.putData(ClosedLoopProximalMove(-5.degree.radian))
        SmartDashboard.putData(ClosedLoopWristMove(0.degree.radian))
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