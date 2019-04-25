package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.team254.drivers.TalonSRXFactory
import frc.robot.Ports
import frc.robot.lib.MotorHelpers
import frc.robot.lib.MultiMotorTransmission
import org.ghrobotics.lib.components.ElevatorComponent
import org.ghrobotics.lib.components.EmergencyHandleable
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.mockito.Mockito

class Elevator(
        override val motor: FalconMotor<Length>,
        override val followerMotors: List<FalconMotor<*>>? = null,
        val pidfSlot: Int,
        override val elevatorKg: Double,
        override val elevatorZero: Translation3d
    ) : ElevatorComponent(elevatorZero), MultiMotorTransmission, EmergencyHandleable {



    override fun activateEmergency(severity: EmergencyHandleable.Severity) {

        when (severity) {
            is EmergencyHandleable.Severity.DisableOutput -> {
                when(motor) {
                    is FalconCTRE -> {
                        motor.motorController.configPeakOutputReverse(0.0, 0)
                        motor.motorController.configPeakOutputForward(0.0, 0)
                    }
                    is FalconMAX -> {
                        motor.controller.setOutputRange(0.0, 0.0)
                    }
                }

                followerMotors?.forEach {
                    when(it) {
                        is FalconCTRE -> {
                            it.motorController.configPeakOutputReverse(0.0, 0)
                            it.motorController.configPeakOutputForward(0.0, 0)
                        }
                        is FalconMAX -> {
                            it.controller.setOutputRange(0.0, 0.0)
                        }
                    }
                }
            }
            is EmergencyHandleable.Severity.DisableClosedLoop -> {
                when(motor) {
                    is FalconCTRE -> {
                        motor.motorController.configClosedLoopPeakOutput(pidfSlot, 0.0, 0)
                    }
                    is FalconMAX -> {
                        motor.controller.setOutputRange(0.0, 0.0)
                    }
                }

                followerMotors?.forEach {
                    when (it) {
                        is FalconCTRE -> {
                            it.motorController.configClosedLoopPeakOutput(pidfSlot, 0.0, 0)
                        }
                        is FalconMAX -> {
                            it.controller.setOutputRange(0.0, 0.0)
                        }
                    }
                }
            }
        }
    }

    override fun recoverFromEmergency() {
        when(motor) {
            is FalconCTRE -> {
                motor.motorController.configPeakOutputReverse(-1.0, 0)
                motor.motorController.configPeakOutputForward(1.0, 0)
                motor.motorController.configClosedLoopPeakOutput(pidfSlot, 1.0, 0)
            }
            is FalconMAX -> {
                motor.controller.setOutputRange(-1.0, 1.0)
            }
        }

        followerMotors?.forEach {
            when(it) {
                is FalconCTRE -> {
                    it.motorController.configPeakOutputReverse(-1.0, 0)
                    it.motorController.configPeakOutputForward(1.0, 0)
                    it.motorController.configClosedLoopPeakOutput(pidfSlot, 1.0, 0)
                }
                is FalconMAX -> {
                    it.controller.setOutputRange(-1.0, 1.0)
                }
            }
        }
    }

    companion object {

        fun getTalonElevator() : Elevator {

            // create the elevator
            val elevatorMotorControllers = listOf(
                    TalonSRXFactory.createDefaultTalon(Ports.SuperStructurePorts.ElevatorPorts.TALON_PORTS[0]),
                    TalonSRXFactory.createPermanentSlaveTalon(Ports.SuperStructurePorts.ElevatorPorts.TALON_PORTS[1], Ports.SuperStructurePorts.ElevatorPorts.TALON_PORTS[0]),
                    TalonSRXFactory.createPermanentSlaveTalon(Ports.SuperStructurePorts.ElevatorPorts.TALON_PORTS[1], Ports.SuperStructurePorts.ElevatorPorts.TALON_PORTS[0]),
                    TalonSRXFactory.createPermanentSlaveTalon(Ports.SuperStructurePorts.ElevatorPorts.TALON_PORTS[1], Ports.SuperStructurePorts.ElevatorPorts.TALON_PORTS[0])
            )

            for (i in 1..3) {
//                elevatorMotorControllers[i].inverted = ElevatorPorts.FOLLOWER_INVERSION[i - 1]
                elevatorMotorControllers[i].setInverted(Ports.SuperStructurePorts.ElevatorPorts.FOLLOWER_INVERSION[i - 1])
            }

            val elevatorFalconSRXes = listOf(
                    FalconSRX(elevatorMotorControllers[0], Ports.SuperStructurePorts.ElevatorPorts.LENGTH_MODEL),
                    FalconSRX(elevatorMotorControllers[1], Ports.SuperStructurePorts.ElevatorPorts.LENGTH_MODEL),
                    FalconSRX(elevatorMotorControllers[2], Ports.SuperStructurePorts.ElevatorPorts.LENGTH_MODEL),
                    FalconSRX(elevatorMotorControllers[3], Ports.SuperStructurePorts.ElevatorPorts.LENGTH_MODEL)
            )

            elevatorFalconSRXes.forEach {
//                MotorHelpers.configureMotor(
//                        it, // etccc
//                )
            }

            elevatorFalconSRXes[0].run {
                outputInverted = Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
                feedbackSensor = Ports.SuperStructurePorts.ElevatorPorts.SENSOR

                feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative

                motorController.setSensorPhase(Ports.SuperStructurePorts.ElevatorPorts.MASTER_SENSOR_PHASE)
                motorController.configPeakOutputForward(1.0, 0)
                motorController.configPeakOutputReverse(-1.0, 0)

            }

            return Elevator(
                    elevatorFalconSRXes[0],
                    listOf(
                            elevatorFalconSRXes[1],
                            elevatorFalconSRXes[2],
                            elevatorFalconSRXes[3]
                    ),
                    0,
                    0.1/9.8,
                    Translation3d(-10.0, 3.0, 0.0)
            )
        }

        fun getMockElevator() : Elevator {
            return Elevator(
                    Mockito.mock(FalconSRX<Length>(0, Ports.SuperStructurePorts.ElevatorPorts.LENGTH_MODEL).javaClass),
                    null,
                    0,
                    1.0,
                    Translation3d(-10.0, 3.0, 0.0)
            )
        }

    }

}