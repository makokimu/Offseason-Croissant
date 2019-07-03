package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.SelectCommand
import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Robot
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearVelocity
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import java.lang.IllegalArgumentException
import java.lang.Math.toDegrees

object SuperStructure: FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    var currentState: State.Position = State.Position()
        private set
    var watedSate: State = State.Nothing
        private set
    private var lastWantedState: State = State.Nothing

    // this Runnable will be run to update the State through a separate Thread periodically
    override fun updateState() {
        // update the states of our components
        Elevator.updateState()
        Proximal.updateState()
        Wrist.updateState()

        // use these updated states to build our current state
        currentState = State.Position(
                Elevator.currentState.position,
                Proximal.currentState.position,
                Wrist.currentState.position,
                wristUnDumb = false
        )
    }

    fun customizeWantedState(state: State): State {

        // todo modify state (ex vision being blocked, illegal state, etc
        return state

    }

    val useState = {
//        val wantedState
    }

    override fun lateInit() {
        Robot.subsystemUpdateList.plusAssign(this)
        SmartDashboard.putData(ElevatorTest())
        SmartDashboard.putData(ProxTest())
        SmartDashboard.putData(WristTest())
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

    override fun recoverFromEmergency() {listOf(Elevator, Proximal, Wrist).forEach { it.recoverFromEmergency() } }

    override fun setNeutral() {
        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral()
    }

    sealed class State {

        object Nothing : State()

        data class Position(
                val elevator: Double,
                val proximal: Double,
                val wrist: Double,
                val isPassedThrough: Boolean = proximal < Math.toRadians(-135.0),
                val isWristUnDumb: Boolean = false
        ): State() {
            @Suppress("unused")
            constructor(elevator:Length, proximal:UnboundedRotation, wrist:UnboundedRotation, isWristUnDumb: Boolean = false
            ) :
                    this(elevator.value, proximal.value, (if(isWristUnDumb) getDumbWrist(wrist, proximal) else wrist).value)

            @Suppress("unused")
            internal constructor(elevator:Double, proximal:Double, wrist:Double, wristUnDumb: Boolean = false
            ) :
                    this(elevator, proximal, (if(wristUnDumb) getDumbWrist(wrist, proximal) else wrist), isWristUnDumb = wristUnDumb)

            constructor() : this(20.inch, 0.degree, 0.degree) // semi-sane numbers?

            fun dumbState() = if(!isWristUnDumb) this else Position(elevator, proximal, getDumbWrist(wrist, proximal))
            fun trueState() = if(isWristUnDumb) this else Position(elevator, proximal, getUnDumbWrist(wrist, proximal))

            fun asString(): String {
                return "True state: Elevator ${elevator/SILengthConstants.kInchToMeter} proximal ${toDegrees(proximal)} wrist ${toDegrees(wrist)}"
            }
        }

        abstract class CustomState : State() {
            abstract fun useState()
        }

    }

}