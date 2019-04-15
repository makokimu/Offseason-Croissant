package frc.robot

object Ports {

    const val kPCMID = 9

    object DrivePorts {
        val LEFT_PORTS = listOf(0, 1)
        val RIGHT_PORTS = listOf(2, 3)
        val SHIFTER_PORTS = listOf(4, 5)
    }

    object IntakePorts {
        val HATCH_PORT = 35
        val CARGO_PORT = 34
        val SHIFTER_PORTS = listOf(0, 1)
    }


}