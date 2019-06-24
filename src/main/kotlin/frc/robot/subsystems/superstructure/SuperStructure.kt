package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.components.*
import org.ghrobotics.lib.mathematics.threedim.geometry.Pose3d
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import kotlin.math.PI

class SuperStructure(
        val elevator: ElevatorComponent,
        val proximal: ArmComponent,
        val wrist: ArmComponent
) : RobotComponent(), EmergencyHandleable {

    init {
        addComponent(elevator)
        elevator.run {
            addComponent(proximal)
        }
        proximal.run {
            addComponent(wrist)
        }
    }

    override fun activateEmergency(severity: EmergencyHandleable.Severity) {
        _children.forEach{
            when(it) {
                is EmergencyHandleable -> {it.activateEmergency(severity)}
            }
        }
    }

    override fun recoverFromEmergency() {
        _children.forEach{
            when(it) {
                is EmergencyHandleable -> {it.recoverFromEmergency()}
            }
        }
    }

    var wantedVisionMode = false

    val position: Preset = Preset(
            elevator.position,
            proximal.position,
            wrist.position
    )

    var wantedState: State = State.Nothing
    var currentState: State = State.Nothing
        private set

    open fun customizeWantedState(wantedState: State): State = wantedState

    override fun updateState() {


        val wantedState = customizeWantedState(wantedState)
        currentState = wantedState
        when (wantedState) {
            is State.Position -> {
                elevator.wantedState = MotorComponent.State.Position(wantedState.setpoint.elevator)
                proximal.wantedState = MotorComponent.State.Position(wantedState.setpoint.proximal)
                wrist.wantedState = MotorComponent.State.Position(wantedState.setpoint.wrist
                        + (wantedState.setpoint.proximal / 2))
            }
        }

        super.updateState()
    }


    @Suppress("unused")
    sealed class State {

        object Nothing : State()

        class Position(val setpoint : Preset) : State() {
            constructor(elevator: Length, proximal: Rotation2d, wrist: Rotation2d) : this(
                    Preset(elevator, proximal, wrist))
            constructor(elevator: Double, proximal: Double, wrist: Double) : this(Preset(elevator, proximal, wrist))
        }

        class PercentOutput(elevatorOutput : Double, proximalOutput: Double, wristOutput: Double ) : State()

    }

    data class Preset(
            val elevator: Double,
            val proximal: Double,
            val wrist: Double
    ) {
        constructor(elevator:Length, proximal:Rotation2d, wrist:Rotation2d)
                : this(elevator.value, proximal.value, wrist.value)
    }

    fun getPresetFromPose3d(wristAngle: Double, pose : Translation3d, isProximalUnder: Boolean,
                            normalizeIfError: Boolean = true): Preset {

//        println("-------------------------------------------------")
//
//        println("input pose: $pose")
//
//        println("elevator zero: ${(elevator.elevatorZero).x}")
//
//        println("proximal arm len: ${proximal.armLength}")

        // figure out displacement from axis to end of proximal linkage
        var proximalDxFromElevator : Double = (pose - elevator.elevatorZero).x


//        println("proximal tip dx from elevator $proximalDxFromElevator")

        // normalize the length from elevator if requested
        if(proximalDxFromElevator > proximal.armLength) {
            if(normalizeIfError) {proximalDxFromElevator = proximal.armLength; println("normalizing dx")} else
                throw IllegalStateException("Cannot extend proximal past max extension")
        }

//        println("dx from elevator $proximalDxFromElevator armlen ${proximal.armLength}")


//        println("that sin thing to figure out proximal: ${Math.acos(proximalDxFromElevator / proximal.armLength)}")

        // figure out proximal angle, with the sign extracted
        val proximalAngle = ( Math.abs(Math.acos(proximalDxFromElevator / proximal.armLength)) *
                (if (isProximalUnder) -1 else 1) ) //% (2 * PI) // idk if the modulo is necessary

//        println("proximal angle: ${Math.toDegrees(proximalAngle)}")

        // determine elevator height based on wanted height and proximal contribution
        val elevatorHeight = pose.y +
                Math.copySign(Math.abs(Math.sin(proximalAngle) * proximal.armLength), proximalAngle)

//        println("arm contribution to vertical height " +
//                "${Math.copySign(Math.abs(Math.sin(proximalAngle) * proximal.armLength), proximalAngle)}")
//
//        println("elevator height: $elevatorHeight")
//
//        println("-------------------------------------------------")

        return Preset(elevatorHeight, proximalAngle, wristAngle)
    }

    companion object {

        val hatchExtended = Preset(10.inch, 0.degree, 0.degree)
    }

}