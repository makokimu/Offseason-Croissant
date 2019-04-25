package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.radian
import org.junit.Test
import org.mockito.Mockito

class ThrustTest {

    @Test
    fun testThrust() {

        val elevator = Elevator.getMockElevator()
        val proximal = Joint.getMockJoint()
        val wrist = Joint.getMockJoint()

        val superStructure = SuperStructure(
                elevator, proximal, wrist
        )

        val thrust = ThrustCommand(
                12.inch.meter, 10.inch.meter, superStructure
        )

        thrust.initialize()

        val traject = thrust.trajectory

        traject.forEach {
            println("elevator ${it.elevator.meter.inch} \t proximal ${it.proximal.radian.degree} \t wrist ${it.wrist.radian.degree}")
        }

    }
}