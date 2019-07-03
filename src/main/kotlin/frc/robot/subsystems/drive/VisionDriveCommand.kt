// implementation from Team 5190 Green Hope Robotics

package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.robot.Network
import frc.robot.Robot
import frc.robot.subsystems.superstructure.SuperStructure
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.radian
import kotlin.math.absoluteValue

class VisionDriveCommand(private val isFront: Boolean) : ManualDriveCommand() {

    override fun isFinished() = false

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        isActive = true
        referencePose = DriveSubsystem.robotPosition
    }

    override fun execute() {

//        val isFront = !SuperStructure.getInstance().isPassedThrough
//        val isFront = 

        println("IS FRONT? $isFront")

        val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, isFront)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        val source = speedSource()

        if (lastKnownTargetPose == null) {
//            ElevatorSubsystem.wantedVisionMode = true
            super.execute()
        } else {
//            ElevatorSubsystem.wantedVisionMode = false
            val transform = lastKnownTargetPose inFrameOfReferenceOf DriveSubsystem.robotPosition
            val angle = Rotation2d(transform.translation.x, transform.translation.y, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val angleError = angle + if (isFront) 0.degree.toRotation2d() else Math.PI.radian.toRotation2d()

            if (angleError.degree.absoluteValue > 45) {
                // plz no disable us when going to loading station, kthx
                this.lastKnownTargetPose = null
            }

            val error = angleError.radian

            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
            DriveSubsystem.tankDrive(source - turn, source + turn)

            prevError = error
        }
    }

    override fun end(interrupted: Boolean) {
        Network.visionDriveActive.setBoolean(false)
        this.lastKnownTargetPose = null
//        ElevatorSubsystem.wantedVisionMode = false
        isActive = false
    }

    override fun initSendable(builder: SendableBuilder) {

        builder.addDoubleProperty("forwardKp", { kCorrectionKp }, { newP -> kCorrectionKp = newP })
        builder.addDoubleProperty("forwardKd", { kCorrectionKd }, { newD -> kCorrectionKd = newD })

        super.initSendable(builder)
    }

    companion object {
        var kCorrectionKp = 0.8
        var kCorrectionKd = 8.0
        var isActive = false
            private set
    }
}