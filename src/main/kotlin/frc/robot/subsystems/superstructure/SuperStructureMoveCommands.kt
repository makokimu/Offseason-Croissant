package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import java.lang.Math.abs
import java.lang.Math.toDegrees

class ClosedLoopElevatorMove(private val target: Double) : FalconCommand(Elevator) {

    override fun initialize() {
        Elevator.wantedState = Elevator.WantedState.Position(target)
    }

    override fun isFinished() = abs(target - Elevator.currentState.position / SILengthConstants.kInchToMeter) < 1.0
}

class ClosedLoopProximalMove(private val target: Double) : FalconCommand(Proximal) {

    override fun initialize() {
        Proximal.wantedState = Proximal.WantedState.Position(target)
    }

    override fun isFinished() = toDegrees(abs(target - Proximal.currentState.position)) < 5.0
}

class ClosedLoopWristMove(private val target: Double) : FalconCommand(Wrist) {

    override fun initialize() {
        Wrist.wantedState = Wrist.WantedState.Position(target)
    }

    override fun isFinished() = toDegrees(abs(target - Wrist.currentState.position)) < 5.0
}