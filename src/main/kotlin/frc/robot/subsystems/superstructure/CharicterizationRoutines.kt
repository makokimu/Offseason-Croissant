package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.commands.FalconCommand

class ElevatorTest: FalconCommand(SuperStructure, Elevator) {

    var oldOutput = 0.0

    override fun initialize() {
        oldOutput = 0.0
        SuperStructure.setNeutral()
    }

    override fun execute() {
        val newOutput = oldOutput + (1.0/5.0)*0.020
        oldOutput=newOutput

        Elevator.setDutyCycle(newOutput, 0.0)

    }

}

class ProxTest: FalconCommand(SuperStructure, Proximal) {

    var oldOutput = 0.0

    override fun initialize() {
        oldOutput = 0.0
    }

    override fun execute() {
        val newOutput = oldOutput + (1.0/5.0)*0.020
        oldOutput=newOutput

        Proximal.setDutyCycle(newOutput, 0.0)

    }

}


class WristTest: FalconCommand(SuperStructure, Wrist) {

    var oldOutput = 0.0

    override fun initialize() {
        oldOutput = 0.0
    }

    override fun execute() {
        val newOutput = oldOutput + (1.0/5.0)*0.020
        oldOutput=newOutput

        Wrist.setDutyCycle(newOutput, 0.0)

    }

}