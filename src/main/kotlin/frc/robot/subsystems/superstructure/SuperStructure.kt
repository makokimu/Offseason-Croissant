package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import edu.wpi.first.wpilibj.experimental.command.SendableSubsystemBase
import frc.robot.auto.routines.AutoRoutine
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.subsystems.EmergencyHandleable

object SuperStructure: SendableSubsystemBase(), EmergencyHandleable {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    // preset semi-singleton commands for each superstructure preset
    val kHatchFrontFromLoadingStation: SendableCommandBase = InstantCommand()

    fun moveTo(state: State): SendableCommandBase {

        return stateCommandGroup(
                // our options are flip carriage or don't flip carriage

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

        enum class 

    }

}