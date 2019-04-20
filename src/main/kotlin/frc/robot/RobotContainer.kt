package frc.robot

import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.superstructure.SuperStructure

class RobotContainer {

    val drive = Drive.getRealTalonDrive()

    val superStructure = SuperStructure.getRealTalonSuperStructure()

    val primary : XboxController = XboxController(0)

    val secondary : XboxController = XboxController(1)


    fun bindButtons() {



    }

}