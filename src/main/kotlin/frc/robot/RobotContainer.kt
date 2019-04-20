package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import frc.robot.lib.AButtonButton
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.superstructure.SuperStructure

class RobotContainer {

    val drive = Drive.getRealTalonDrive()

    val superStructure = SuperStructure.getRealTalonSuperStructure()

    val primary : XboxController = XboxController(0)

    val secondary : XboxController = XboxController(1)

    fun bindButtons() {

        drive.defaultCommand = drive.curvatureDriveCommand(
                {primary.getY(GenericHID.Hand.kLeft)},
                {primary.getX(GenericHID.Hand.kRight) * -1},
                {primary.getBumper(GenericHID.Hand.kLeft) || primary.getBumper(GenericHID.Hand.kRight)}
        )
    }
}