package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import frc.robot.Robot
import kotlinx.coroutines.channels.Channel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.FalconChannel
import org.team5940.pantry.lib.WantedState
import java.lang.Math.toDegrees

object Superstructure : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    val kStowed
            get() = everythingMoveTo(29.inch, (-70).degree, 36.degree)

    val kBackHatchFromLoadingStation
            get() = SyncedMove.frontToBack
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

    fun everythingMoveTo(goalstate: SuperstructureState) = SuperstructurePlanner.everythingMoveTo(goalstate)

    fun getUnDumbWrist(dumbWrist: UnboundedRotation, relevantProx: UnboundedRotation) =
            dumbWrist.plus(relevantProx.div(2))

    fun getUnDumbWrist(dumbWrist: Double, relevantProx: Double) =
            dumbWrist.plus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: UnboundedRotation, relevantProx: UnboundedRotation) =
            smartWrist.minus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: Double, relevantProx: Double) =
            smartWrist - (relevantProx / 2)

    override fun activateEmergency() {
//        listOf(Elevator, Proximal, Wrist).forEach { it.activateEmergency() }
        Elevator.activateEmergency()
        Proximal.activateEmergency()
        Wrist.activateEmergency()
    }

    override fun recoverFromEmergency() {
        Elevator.recoverFromEmergency()
        Proximal.recoverFromEmergency()
        Wrist.recoverFromEmergency()
//        listOf(Elevator, Proximal, Wrist).forEach { it.recoverFromEmergency() }
        }

    override fun setNeutral() {
        Elevator.wantedState = WantedState.Nothing
        Proximal.wantedState = WantedState.Nothing
        Wrist.wantedState = WantedState.Nothing

        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral()
    }

    var currentState: SuperstructureState = SuperstructureState()
        @Synchronized get
        @Synchronized set

    override fun periodic() {
        SmartDashboard.putString("Superstructurestate", currentState.asString())
    }

    override fun updateState() {
        // update the states of our components
        Elevator.updateState()
        Proximal.updateState()
        Wrist.updateState()

        // use these updated states to build our current state
        val newState = State.Position(
            Elevator.currentState.position,
            Proximal.currentState.position,
            Wrist.currentState.position,
            wristUnDumb = false
        )
        synchronized(currentState) {
            currentState = newState
        }
}

    override fun useState() {
        Elevator.useState()
        Proximal.useState()
        Wrist.useState()
    }

    override fun lateInit() {
        Proximal.resetPosition(0)
        Wrist.resetPosition(0)

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

            fun proximalTranslation() =
                    Translation2d(kProximalLen.meter, proximal.radian.toRotation2d())

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

typealias SuperstructureState = Superstructure.State.Position
