package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.team254.drivers.TalonSRXFactory
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.lib.RoundRotation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.exparimental.command.SendableSubsystemBase

class SuperStructure (
        elevator : Joint<Length>,
        proximal : Joint<RoundRotation2d>,
        wrist: Joint<RoundRotation2d>
) : SendableSubsystemBase() {




    companion object {

        fun getRealTalonSuperStructure() {

            val elevatorMotorControllers = listOf(
                    TalonSRXFactory.createDefaultTalon(ElevatorPorts.TALON_PORTS[0]),
                    TalonSRXFactory.createPermanentSlaveTalon(ElevatorPorts.TALON_PORTS[1], ElevatorPorts.TALON_PORTS[0]),
                    TalonSRXFactory.createPermanentSlaveTalon(ElevatorPorts.TALON_PORTS[1], ElevatorPorts.TALON_PORTS[0]),
                    TalonSRXFactory.createPermanentSlaveTalon(ElevatorPorts.TALON_PORTS[1], ElevatorPorts.TALON_PORTS[0])
            )

            for (i in 1..3) {
//                elevatorMotorControllers[i].inverted = ElevatorPorts.FOLLOWER_INVERSION[i - 1]
                elevatorMotorControllers[i].setInverted(ElevatorPorts.FOLLOWER_INVERSION[i - 1])
            }

            val elevatorTalons = listOf(
                    FalconSRX(elevatorMotorControllers[0], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX(elevatorMotorControllers[1], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX(elevatorMotorControllers[2], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX(elevatorMotorControllers[3], ElevatorPorts.LENGTH_MODEL)
            )

            elevatorTalons[0].run {
                outputInverted = ElevatorPorts.MASTER_INVERTED
                feedbackSensor = ElevatorPorts.SENSOR

                feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative

                motorController.setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)
                motorController.configPeakOutputForward(1.0, 0)
                motorController.configPeakOutputReverse(-1.0, 0)

            }



        }

    }

}