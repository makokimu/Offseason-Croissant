 package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.* // ktlint-disable no-wildcard-imports
import kotlin.math.roundToInt

typealias Length = SIUnit<Meter>

object Superstructure : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    val kStowed
        get() = everythingMoveTo(30.25.inches, (-70).degrees, 40.degrees)
    val kMatchStartToStowed get() = sequential {
        +parallel {
            +ClosedLoopProximalMove((-70).degrees)
            +ClosedLoopWristMove(40.degrees)
        }
        +ClosedLoopElevatorMove(30.25.inches)
        +kStowed
    }
    val kBackHatchFromLoadingStation get() = SyncedMove.frontToBack
    val kHatchLow get() = everythingMoveTo(19.inches, 0.degrees, 4.degrees)
    val kHatchMid get() = everythingMoveTo(45.inches, 0.degrees, 4.degrees)
    val kHatchHigh get() = everythingMoveTo(67.inches, 0.degrees, 4.degrees)

    val kCargoIntake get() = everythingMoveTo(25.5.inches, (-44).degrees, (-20).degrees)
    val kCargoShip get() = everythingMoveTo(47.5.inches, (-5).degrees, (-50).degrees)
    val kCargoLow get() = everythingMoveTo(20.5.inches, 6.degrees, 16.degrees)
    val kCargoMid get() = everythingMoveTo(45.5.inches, 6.degrees, 16.degrees)
    val kCargoHigh get() = everythingMoveTo(64.5.inches, 7.degrees, 30.degrees)

    fun everythingMoveTo(elevator: Length, proximal: SIUnit<Radian>, wrist: SIUnit<Radian>) = everythingMoveTo(State.Position(elevator, proximal, wrist))

    fun everythingMoveTo(goalState: SuperstructureState) = SuperstructurePlanner.everythingMoveTo(goalState)

    fun getUnDumbWrist(dumbWrist: SIUnit<Radian>, relevantProx: SIUnit<Radian>) =
            dumbWrist.plus(relevantProx.div(2))

    fun getUnDumbWrist(dumbWrist: Double, relevantProx: Double) =
            dumbWrist.plus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: SIUnit<Radian>, relevantProx: SIUnit<Radian>) =
            smartWrist.minus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: Double, relevantProx: Double) =
            smartWrist - (relevantProx / 2)

    override fun activateEmergency() {
        Elevator.activateEmergency()
        Proximal.activateEmergency()
        Wrist.activateEmergency() }

    override fun recoverFromEmergency() {
        Elevator.recoverFromEmergency()
        Proximal.recoverFromEmergency()
        Wrist.recoverFromEmergency() }

    override fun setNeutral() {
        Elevator.wantedState = WantedState.Nothing
        Proximal.wantedState = WantedState.Nothing
        Wrist.wantedState = WantedState.Nothing

        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral() }

    private val currentStateMutex = Object()
    var currentState = SuperstructureState()
        get() = synchronized(currentStateMutex) { field }
        set(newValue) = synchronized(currentStateMutex) { field = newValue }

    override fun updateState() {
        // update the states of our components

        Wrist.updateState()

        // use these updated states to build our current state
        val newState = State.Position(
            Elevator.updateState().position,
            Proximal.updateState().position,
            Wrist.currentState.position
//            wristUnDumb = false
        )

        currentState = newState
    }

    override fun useState() {
        Elevator.useState()
        Proximal.useState()
        Wrist.useState()
    }

    val zero = ZeroSuperStructureRoutine()
    override fun lateInit() {
        Proximal.zero()
        Wrist.zero()

        SmartDashboard.putData(zero)
        zero.schedule()

        SmartDashboard.putData(this)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addStringProperty("cState", { currentState.asString() }, { })
        builder.addDoubleProperty("cProxTranslation", { currentState.proximalTranslation().y.inches }, { })
        super.initSendable(builder)
    }

    sealed class State {

        object Nothing : State()

        data class Position(
                val elevator: SIUnit<Meter>,
                val proximal: SIUnit<Radian>,
                val wrist: SIUnit<Radian>,
                val isPassedThrough: Boolean = proximal < (-135).degrees,
                val isWristUnDumb: Boolean = false
        ) : State() {

            constructor() : this(20.inches, (-90).degrees, (-45).degrees) // semi-sane numbers?

            fun proximalTranslation() =
                    Translation2d(kProximalLen, proximal.toRotation2d()) + Translation2d(0.meters, elevator)

            fun dumbState() = if (!isWristUnDumb) this else Position(elevator, proximal, getDumbWrist(wrist, proximal))
            fun trueState() = if (isWristUnDumb) this else Position(elevator, proximal, getUnDumbWrist(wrist, proximal))

            fun asString(): String {
                return "Elevator [${(elevator.inch).roundToInt()}\"] proximal [${proximal.degree.roundToInt()}deg] wrist [${wrist.degree.roundToInt()}deg]"
            }

            override fun toString(): String = asString()
        }

        abstract class CustomState : State() {
            abstract fun useState()
        }
    }
}

typealias SuperstructureState = Superstructure.State.Position
