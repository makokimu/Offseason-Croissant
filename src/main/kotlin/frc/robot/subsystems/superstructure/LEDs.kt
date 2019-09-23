package frc.robot.subsystems.superstructure

import com.ctre.phoenix.CANifier
import org.ghrobotics.lib.commands.FalconSubsystem
import java.awt.Color

object LEDs: FalconSubsystem() {

    override fun lateInit() {
        setColor(PURPLE)
    }

    private val canifier by lazy { Proximal.canifier }
    fun setColor(color: Color) {
        canifier.setLEDOutput(color.red * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelB)
        canifier.setLEDOutput(color.green * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelA)
        canifier.setLEDOutput(color.blue * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelC)
    }
    val PURPLE = Color(0, 125, 125)
}