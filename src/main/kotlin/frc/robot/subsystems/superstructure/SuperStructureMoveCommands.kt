package frc.robot.subsystems.superstructure

import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import java.lang.Math.abs
import java.lang.Math.toDegrees

class ClosedLoopElevatorMove(private val target: Double) : FalconCommand(Elevator) {

    override fun initialize() {
        Elevator.wantedState = Elevator.WantedState.Position(target)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // stop moving
            Elevator.wantedState = Elevator.WantedState.Position(Elevator.currentState.position)
        }
    }

    override fun isFinished(): Boolean {
        val error = abs(target - Elevator.currentState.position) / SILengthConstants.kInchToMeter
        return error < 1.0
    }
}

class ClosedLoopProximalMove(private val target: Double) : FalconCommand(Proximal) {

    override fun initialize() {
        Proximal.wantedState = Proximal.WantedState.Position(target)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // stop moving
            Proximal.wantedState = Proximal.WantedState.Position(Proximal.currentState.position)
        }
    }

    override fun isFinished() = toDegrees(abs(target - Proximal.currentState.position)) < 5.0
}

class ClosedLoopWristMove(private val target: Double) : FalconCommand(Wrist) {

    override fun initialize() {
        Wrist.wantedState = Wrist.WantedState.Position(target)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // stop moving
            Wrist.wantedState = Wrist.WantedState.Position(Wrist.currentState.position)
        }
    }

    override fun isFinished() = toDegrees(abs(target - Wrist.currentState.position)) < 5.0
}

class JogElevator : FalconCommand(Superstructure, Elevator) {

    companion object {
        val upSource by lazy { (Controls.operatorFalconHID.getRawButton(9)) }
        val downSource by lazy { (Controls.operatorFalconHID.getRawButton(11)) }
    }

    override fun execute() {
        val upPower = if (upSource()) 1 else -1
        val downPower = if (downSource()) -1 else 1
        val totalPower = (upPower + downPower) * 0.3 * SILengthConstants.kInchToMeter

        Elevator.wantedState = Elevator.WantedState.Position(
                Elevator.currentState.position + totalPower
        )
    }
}