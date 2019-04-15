package frc.robot

import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.drive.Drive

class RobotContainer {

    val drive = Drive.createNewTalonDrive()

    val primary : XboxController = XboxController(0)

    val secondary : XboxController = XboxController(1)


    fun bindButtons() {



    }

}