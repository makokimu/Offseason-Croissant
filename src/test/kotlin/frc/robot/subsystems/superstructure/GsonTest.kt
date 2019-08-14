package frc.robot.subsystems.superstructure

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.JsonElement
import com.google.gson.JsonObject
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.junit.Test
import org.team5940.pantry.lib.jsonToTrajectory
import org.team5940.pantry.lib.kGson
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