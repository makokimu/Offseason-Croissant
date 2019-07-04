
package frc.robot

import edu.wpi.first.wpilibj.Notifier
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.superstructure.SuperStructure
import frc.robot.vision.JeVoisManager
import frc.robot.vision.LimeLightManager
import frc.robot.vision.TargetTracker
import frc.robot.vision.VisionProcessing
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import frc.robot.subsystems.superstructure.Wrist
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.radian

object Robot : FalconTimedRobot() {

  private lateinit var stateUpdater: Notifier
  private lateinit var stateUser: Notifier
  var subsystemUpdateList = arrayListOf<ConcurrentlyUpdatingComponent>()
    @Synchronized get

  override fun robotInit() {
    +DriveSubsystem
    +SuperStructure
    +Intake

    TargetTracker
    JeVoisManager
    LimeLightManager
    VisionProcessing
    Controls

    stateUpdater = Notifier { subsystemUpdateList.forEach { it.updateState() } }
    stateUpdater.startPeriodic(1.0 / 50.0)

    stateUser = Notifier { subsystemUpdateList.forEach { it.useState() } }
    stateUser.startPeriodic(1.0 / 50.0)

    Wrist.position = ((-90).degree.radian)
    Wrist.encoder.resetPosition(Wrist.master.model.toNativeUnitPosition((-90).degree.radian))
    Wrist.position = ((-90).degree.radian)
    Wrist.encoder.resetPosition(Wrist.master.model.toNativeUnitPosition((-90).degree.radian))
    Wrist.position = ((-90).degree.radian)
    Wrist.encoder.resetPosition(Wrist.master.model.toNativeUnitPosition((-90).degree.radian))
  }

  override fun robotPeriodic() {
    TargetTracker.update()
    Controls.update()
  }

  override fun disabledInit() {
  }

//  var autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
//  var telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")
//  var priorAutospeed = 0.0
//  var numberArray = arrayOfNulls<Number>(6)
//
//  override fun autonomousInit() {
//    Elevator.encoder.resetPosition(0.0)
//    Proximal.position = ((-90).degree.radian)
//    Wrist.position = ((-90).degree.radian)
//    Wrist.encoder.resetPosition(Wrist.master.model.toNativeUnitPosition((-90).degree.radian))
//  }
//
//  /**
//   * If you wish to just use your own robot program to use with the data logging
//   * program, you only need to copy/paste the logic below into your code and
//   * ensure it gets called periodically in autonomous mode
//   *
//   * Additionally, you need to set NetworkTables update rate to 10ms using the
//   * setUpdateRate call.
//   */
//  override fun autonomousPeriodic() {
//
//    // Retrieve values to send back before telling the motors to do something
//    val now = getFPGATimestamp()
//
//    val position = Wrist.position.radian.degree
//    val rate = Wrist.currentState.velocity.radian.degree
//
//    val battery = RobotController.getBatteryVoltage()
//
//    val motorVolts = Wrist.voltageOutput
//
//    // Retrieve the commanded speed from NetworkTables
//    val autospeed = autoSpeedEntry.getDouble(0.0)
//    priorAutospeed = autospeed
//
//    // command motors to do things
// //    println("setting prox to $autospeed")
//    Wrist.setDutyCycle(autospeed, 0.0)
// //    armMotor.set(autospeed)
//
// //    println("reporting position $position")
//
//    // send telemetry data array back to NT
//    numberArray[0] = now
//    numberArray[1] = battery
//    numberArray[2] = autospeed
//    numberArray[3] = motorVolts
//    numberArray[4] = position
//    numberArray[5] = rate
//    telemetryEntry.setNumberArray(numberArray)
//  }
}

fun main() {
  Robot.start()
}