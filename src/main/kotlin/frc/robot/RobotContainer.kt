package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import frc.robot.lib.AButtonButton
import frc.robot.lib.XButtonButton
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.SuperStructure

class RobotContainer {

    val drive = Drive.getRealTalonDrive()

//    val superStructure = SuperStructure(Elevator.getTalonElevator(), SuperStructure.getProximal(), SuperStructure.getWrist(), )

    init {
//        drive.addComponent(superStructure)
    }

    val primary : XboxController = XboxController(0)

    val secondary : XboxController = XboxController(1)

    fun bindButtons() {

        drive.defaultCommand = drive.curvatureDriveCommand(
                {primary.getY(GenericHID.Hand.kLeft)},
                {primary.getX(GenericHID.Hand.kRight) * -1},
                {primary.getBumper(GenericHID.Hand.kLeft) || primary.getBumper(GenericHID.Hand.kRight)}
        )

        primary.XButtonButton.whenPressed(Runnable{drive.lowGear = true})
        primary.AButtonButton.whenPressed(Runnable{drive.lowGear = false})

    }
}