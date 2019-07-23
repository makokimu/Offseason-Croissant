
package frc.robot

import edu.wpi.first.wpilibj.experimental.command.CommandScheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.auto.Autonomous
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.Proximal
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.vision.JeVoisManager
import frc.robot.vision.LimeLightManager
import frc.robot.vision.TargetTracker
import frc.robot.vision.VisionProcessing
import frc.robot.subsystems.superstructure.Wrist
import org.team5940.pantry.lib.FishyRobot

object Robot : FishyRobot() {

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated

        +DriveSubsystem
        +Proximal
        +Wrist
        +Elevator
        +Superstructure
        +Intake

        TargetTracker
        JeVoisManager
        LimeLightManager
        VisionProcessing
        Controls
        Autonomous

        SmartDashboard.putData(CommandScheduler.getInstance())

        subsystemUpdateList.add(DriveSubsystem)
        subsystemUpdateList.add(Superstructure)

        super.robotInit()
    }

    override fun robotPeriodic() {
        TargetTracker.update()
        Controls.update()
        Autonomous.update()
        Network.update()
        super.robotPeriodic()
    }
}

fun main() {
    Robot.start()
}