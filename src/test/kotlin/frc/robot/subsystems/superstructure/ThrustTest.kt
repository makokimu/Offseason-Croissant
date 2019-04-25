package frc.robot.subsystems.superstructure

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
                12.toDouble(), 10.toDouble(), superStructure
        )

        thrust.initialize()

        val traject = thrust.trajectory

        traject.forEach {
            println(it)
        }

    }
}