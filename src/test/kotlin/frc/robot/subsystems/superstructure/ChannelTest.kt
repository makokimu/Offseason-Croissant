package frc.robot.subsystems.superstructure

import org.junit.Assert.*
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.runBlocking
import org.junit.Test
import org.team5940.pantry.lib.FalconChannel

class ChannelTest {

    @Test
    fun testChannels() {

        val currentStateChannel = FalconChannel(0, capacity = -1)

        // test multi input
        runBlocking {
            for(i in 1..100) {
                currentStateChannel.send(i)
            }
        }

        val received0 = runBlocking { currentStateChannel.receive() }
        println("received0 is $received0")

        val received1 = runBlocking { currentStateChannel.receive() }
        println("received1 is $received1")


    }

}