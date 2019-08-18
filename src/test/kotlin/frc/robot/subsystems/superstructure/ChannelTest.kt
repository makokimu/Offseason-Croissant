// package frc.robot.subsystems.superstructure
//
// import kotlinx.coroutines.channels.Channel
// import org.junit.Test
// import org.team5940.pantry.lib.FalconConflatedChannel
//
// class ChannelTest {
//
//    @Test
//    fun test() {
//
//        val channel = FalconConflatedChannel(0)
//        channel.offer(1)
//        channel.offer(3)
//        channel.poll()
//        channel.offer(2)
//        for(i in 1..300) channel.poll()
//        val toRet2 = channel.poll()
//        println("recieved2 $toRet2")
//
//
//
//
//    }
//
// }