package frc.robot.subsystems.drive

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Network
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.absoluteValue

class ClosedLoopVisionDriveCommand(private val isFront: Boolean) : FalconCommand(DriveSubsystem) {

    override fun isFinished() = false

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        isActive = true
        referencePose = DriveSubsystem.robotPosition
        LEDs.wantedState = LEDs.State.Off
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0)
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0)
    }

    override fun execute() {
        val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, isFront)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        val source = -ManualDriveCommand.speedSource()

        if (lastKnownTargetPose == null) {
            super.execute()
        } else {
            val transform = lastKnownTargetPose inFrameOfReferenceOf DriveSubsystem.robotPosition
            val angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)
            val distance = transform.translation.norm.feet

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val angleError = angle + if (isFront) 0.degree.toRotation2d() else Math.PI.radian.toRotation2d() - 1.7.degree.toRotation2d()

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
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1)
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1)
    }

    override fun initSendable(builder: SendableBuilder) {
//        builder.addDoubleProperty("Kp", {kCorrectionKd}, { value -> kCorrectionKp = value})
//        builder.addDoubleProperty("Kd", {kCorrectionKd}, { value -> kCorrectionKd = value})
    }

    companion object {
        var kCorrectionKp = 12.0
        var kCorrectionKd = 0.0
        var isActive = false
            private set
    }

    init {
        SmartDashboard.putData(this)
    }
}