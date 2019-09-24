// implementation from Team 5190 Green Hope Robotics

package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.robot.Network
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.second
import java.awt.Color
import kotlin.math.absoluteValue

class VisionDriveCommand(private val isFront: Boolean) : ManualDriveCommand() {

    override fun isFinished() = false

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        isActive = true
        referencePose = DriveSubsystem.robotPosition
        LEDs.setVisionMode(true)
    }

    override fun execute() {

//        val isFront = !Superstructure.getInstance().isPassedThrough
//        val isFront = 

        println("IS FRONT? $isFront")

        val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, isFront)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        val source = -speedSource()

        if (lastKnownTargetPose == null) {
//            ElevatorSubsystem.wantedVisionMode = true
            super.execute()
        } else {
//            ElevatorSubsystem.wantedVisionMode = false
            val transform = lastKnownTargetPose inFrameOfReferenceOf DriveSubsystem.robotPosition
            val angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)
            val distance = transform.translation.norm.feet
            // so when we're 4ft away we want a blink freq of like 2x/sec
            // and when we're 1.5ft away like 6x per sec
            // so y = -1.6x + 8.4
            val frequency = -1.6 * distance + 8.4
//            LEDs.blinkFreq = 1.second / frequency
            LEDs.wantedState = LEDs.State.Blink((1.0/frequency).second, Color.GREEN)

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
        LEDs.setVisionMode(false)
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