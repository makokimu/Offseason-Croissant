package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import frc.robot.lib.RoundRotation2d
import org.ghrobotics.lib.mathematics.units.*
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import org.ghrobotics.lib.motors.ctre.FalconSRX

class SuperStructure (
        elevator : Joint<Length>,
        proximal : Joint<RoundRotation2d>
) {




    companion object {

        fun getRealTalonSuperStructure() {

            val elevatorTalons = listOf(
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[0], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[1], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[2], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[3], ElevatorPorts.LENGTH_MODEL)
            )

            elevatorTalons[0].run {
                outputInverted = ElevatorPorts.MASTER_INVERTED
//                inverted = ElevatorPorts.MASTER_INVERTED
                feedbackSensor = ElevatorPorts.SENSOR

                feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative

                motorController.setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)
                motorController.configPeakOutputForward(1.0, 0)
                motorController.configPeakOutputReverse(-1.0, 0)

            }
//            elevatorTalons[0].inverted = ElevatorPorts.MASTER_INVERTED
//            elevatorTalons[0].feedbackSensor = ElevatorPorts.SENSOR
//            elevatorTalons[0].setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)

            for (i in 1..3) {
                elevatorTalons[i].motorController.set(ControlMode.Follower, elevatorTalons[0].motorController.deviceID.toDouble())
                // xd a breaking change TODO meme
//                elevatorTalons[i].motorController.setInverted(ElevatorPorts.FOLLOWER_INVERSION[i - 1])
            }

        }

    }

}