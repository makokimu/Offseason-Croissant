/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import edu.wpi.first.wpilibj.Notifier
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.vision.JeVoisManager
import frc.robot.vision.LimeLightManager
import frc.robot.vision.TargetTracker
import frc.robot.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
object Robot : FalconTimedRobot() {

  private lateinit var stateUpdater: Notifier
  private lateinit var stateUser: Notifier
  var subsystemUpdateList = arrayListOf<ConcurrentlyUpdatingComponent>()
    @Synchronized get

  override fun robotInit() {
    +DriveSubsystem

    TargetTracker
    JeVoisManager
    LimeLightManager
    VisionProcessing

    stateUpdater = Notifier { subsystemUpdateList.forEach { it.updateState() } }
    stateUpdater.startPeriodic(1.0/100.0)

    stateUser = Notifier { subsystemUpdateList.forEach { it.useState() } }
    stateUser.startPeriodic(1.0/100.0)

  }

  override fun robotPeriodic() {
    TargetTracker.update()
  }

}

fun main() {
  Robot.start()
}