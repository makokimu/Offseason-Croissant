package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants
import frc.robot.subsystems.sensors.LimeLight
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.inch

class AutonomousVisionDriveCommand(val isStowed: Boolean, val skewCorrect: Boolean = true): FalconCommand(DriveSubsystem) {

    var noTarget = 0
    var cantFindTarget = false
    var inRange = false
    var inRangeTime = -1.0
    val kEndTimeout = 0.6
    private var prevAngleError = 0.degree.toRotation2d()
    private val targetDistance = (if(isStowed) Constants.kCenterToForwardIntakeStowed else Constants.kCenterToForwardIntake).translation.x.absoluteValue

    override fun end(interrupted: Boolean) {
        noTarget = 0
        cantFindTarget = false
        inRange = false
        inRangeTime = -1.0
        prevAngleError = 0.degree.toRotation2d()
    }

    override fun initialize() {
        noTarget = 0
        cantFindTarget = false
        inRange = false
        prevAngleError = 0.degree.toRotation2d()
    }

    override fun execute() {
        if(!LimeLight.hasTarget) {
            noTarget++
            if(noTarget > 40) cantFindTarget = true
            return
        }
        // we know we have a new target
        noTarget = 0
        var offset = 0.degree
        var skew = LimeLight.lastSkew
        if(skew > (-45).degree) skew = skew.absoluteValue else skew += 90.degree
        if(skew > 5.degree) offset = 0.05.degree * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13);
        if(!skewCorrect) offset = 0.degree

        val globalPose = (LimeLight.lastYaw - offset).toRotation2d() + DriveSubsystem.localization[LimeLight.currentState.timestamp].rotation
        val angleError = (globalPose - DriveSubsystem.robotPosition.rotation)
        // idk man maybe 1 feet per second at 1 ft of error?
        val currentDistance = LimeLight.estimateDistance(false)
        val linear = (currentDistance - Constants.kCenterToFrontCamera.translation.x - targetDistance).velocity * 1.0 // TODO tune

        // P loop on heading
        val turn = PointTurnCommand.kCorrectionKp * angleError.radian + PointTurnCommand.kCorrectionKd * (angleError - prevAngleError).radian

        DriveSubsystem.setWheelVelocities(DifferentialDrive.WheelState(linear.value + turn, linear.value - turn))

        inRange = currentDistance < targetDistance + 2.inch
        if(inRange && inRangeTime > 0.0) inRangeTime = Timer.getFPGATimestamp()

        prevAngleError = angleError
    }

    override fun isFinished() = cantFindTarget || (inRange && (inRangeTime - Timer.getFPGATimestamp()) > kEndTimeout)


}