package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import edu.wpi.first.wpilibj.experimental.command.SendableSubsystemBase
import frc.robot.auto.routines.AutoRoutine
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.subsystems.EmergencyHandleable

object SuperStructure: SendableSubsystemBase(), EmergencyHandleable {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    // preset semi-singleton commands for each superstructure preset
    val kHatchFrontFromLoadingStation: SendableCommandBase = everythingMoveTo(State(20.inch, 0.degree, 0.degree))

    fun everythingMoveTo(state: State) = stateCommandGroup(
                // our first parameter is a supplier which decides between the 4 states
                // kinda like a planner i guess
                { /* TODO FIX ME */ MovementType.ARM_THEN_ELEVATOR },

                // our second parameter is a runnable which is run on init to add the states
                // these InstantCommands would need to be replaced with a command to actually
                // do the thing that the states say (ex. ARM_THEN_ELEVATOR needs to actually do that)
                {
                    state(MovementType.ARM_THEN_ELEVATOR, InstantCommand())
                    state(MovementType.ELEVATOR_THEN_ARM, InstantCommand())
                    state(MovementType.FRONT_TO_BACK, InstantCommand())
                    state(MovementType.BACK_TO_FRONT, InstantCommand())

                }

        )

    }

    override fun activateEmergency() {
        listOf(Elevator, Proximal, Wrist).forEach { it.activateEmergency() }
    }

    override fun recoverFromEmergency() {
        listOf(Elevator, Proximal, Wrist).forEach { it.recoverFromEmergency() }
    }

    data class State(
            val elevator: Double,
            val proximal: Double,
            val wrist: Double,
            val isPassedThrough: Boolean = proximal < Math.toRadians(-135.0)
    ) {
        @Suppress("unused")
        constructor(elevator:Length, proximal:UnboundedRotation, wrist:UnboundedRotation) :
                this(elevator.value, proximal.value, wrist.value)
    }

    @Suppress("unused")
    enum class MovementType {
        ARM_THEN_ELEVATOR,
        ELEVATOR_THEN_ARM,
        FRONT_TO_BACK,
        BACK_TO_FRONT
    }

}