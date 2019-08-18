package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.junit.Test
import org.team5940.pantry.lib.toJson

class GsonTest {

    val test: Double
        get() = Math.random()

    @Test
    fun testGson() {

        val trajectory = DefaultTrajectoryGenerator.baseline

        val jsonedTrajectory = trajectory.toJson()

//        val unJsoned = jsonToTrajectory(jsonedTrajectory)

//        println(unJsoned)
    }
}