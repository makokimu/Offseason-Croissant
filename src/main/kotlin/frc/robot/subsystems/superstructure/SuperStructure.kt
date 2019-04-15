package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import frc.robot.lib.RoundRotation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import org.team5940.pantry.experimental.command.SendableSubsystemBase
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts

class SuperStructure (
        elevator : Joint<Length>,
        proximal : Joint<RoundRotation2d>
) {




    companion object {

        fun getRealSuperStructure() {

            val elevatorTalons = listOf(
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[0], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[1], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[2], ElevatorPorts.LENGTH_MODEL),
                    FalconSRX<Length>(ElevatorPorts.TALON_PORTS[3], ElevatorPorts.LENGTH_MODEL)
            )

            elevatorTalons[0].run {
                inverted = ElevatorPorts.MASTER_INVERTED
                feedbackSensor = ElevatorPorts.SENSOR
                setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)
                peakForwardOutput = 1.0
                peakReverseOutput = -1.0

            }
//            elevatorTalons[0].inverted = ElevatorPorts.MASTER_INVERTED
//            elevatorTalons[0].feedbackSensor = ElevatorPorts.SENSOR
//            elevatorTalons[0].setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)

            for (i in 1..3) {
                elevatorTalons[i].set(ControlMode.Follower, elevatorTalons[0].deviceID)
                elevatorTalons[i].setInverted(ElevatorPorts.FOLLOWER_INVERSION[i - 1])
            }

        }

    }

}