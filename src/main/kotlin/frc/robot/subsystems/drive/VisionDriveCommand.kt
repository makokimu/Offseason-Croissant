package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.robot.Network
import frc.robot.vision.LimeLight
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.radian
import kotlin.math.abs

class VisionDriveCommand(val side: TargetSide) : ManualDriveCommand() {

    enum class TargetSide {
        FRONT, BACK
    }

    private var lemonLightHasTarget: Boolean = false
    private var lastKnownAngle: Double = 0.0
    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        referencePose = DriveSubsystem.robotPosition
    }

    override fun execute() {

        var isFront = side == TargetSide.FRONT//!SuperStructure.getInstance().isPassedThrough

//        isFront = false // for debugging

        var turnInput: Double?

        println("looking for vision targets on the ${if (isFront) "front" else "back"}")

        if (isFront) {
            // it's LimeLight Time
            val lemonLight = LimeLight
            val hasTarget = lemonLight.trackedTargets > 0.5

            this.lemonLightHasTarget = hasTarget

            // check that we have a target
            if (!hasTarget) {
                turnInput = null
                println("no vision targets found!")
            } else {
                println("limelight found vision target!")
                val dx = lemonLight.dx.degree
                turnInput = dx
            }
        } else {

            // is YeeVois Time
            val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, false)

//            println("is the back jevois even connected? ${JeVoisManager.isBackJeVoisConnected}")

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

            val lastKnownTargetPose = this.lastKnownTargetPose
            if (lastKnownTargetPose == null) {
                turnInput = null
                println("no vision targets found!")
            } else {
                println("jevois target found!")
                val transform = lastKnownTargetPose inFrameOfReferenceOf DriveSubsystem.robotPosition // TODO check math
                var angle = Rotation2d(transform.translation.x, transform.translation.y, true)

                // since it's the back i don't care

                if (angle.degree < -90) angle = angle.plus(180.degree)

                if (angle.degree > 90) angle = angle.minus(180.degree)

                Network.visionDriveAngle.setDouble(angle.degree)
                Network.visionDriveActive.setBoolean(true)

                val angleError = angle + if (isFront) Rotation2d() else Math.PI.radian.toRotation2d()


                turnInput = angleError.degree

            }
        }

        // check if our vision even sees anything - if not, normal drive time
        if (turnInput == null) {
            println("no target, going to default execute method")
            super.execute()
        } else {

            this.lastKnownAngle = turnInput

            var forward = speedSource()
            forward *= abs(forward)
            if (DriveSubsystem.lowGear) {
                forward *= 0.8
            }

            println("angle error $turnInput")

            println("kp $kp_mutable kd $kd_mutable")
            var turn = if (isFront) {
                println("limelightPID")
                kLemonLightkP * turnInput - kLemonLightkD * (turnInput - prevError)
            } else {
                println("jevoisPID")
                kJevoiskP * turnInput - kJevoiskD * (turnInput - prevError)
            }

            if (turn > 0.6) turn = 0.6
            if (turn < -0.6) turn = -0.6

            println("Commanding state $forward, $turn")

            DriveSubsystem.arcadeDrive(forward, turn)

            prevError = turnInput
        }
    }

    private var kp_mutable = kJevoiskP
    private var kd_mutable = kJevoiskD

    override fun initSendable(builder: SendableBuilder) {

        builder.addDoubleProperty("angle", { Math.toDegrees(lastKnownAngle) }, null)

        builder.addDoubleProperty("kp", { kp_mutable }, {
            this.kp_mutable = it
        })

        builder.addDoubleProperty("kd", { kd_mutable }, {
            this.kd_mutable = it
        })

        super.initSendable(builder)
    }

    companion object {
        const val kJevoiskP = 0.002
        const val kJevoiskD = 0.04
        const val kLemonLightkP = 0.04
        const val kLemonLightkD = 0.0
    }

    override fun isFinished() = false
}

operator fun Rotation2d.plus(other: UnboundedRotation) = Rotation2d(this@plus.value + other.value)
operator fun Rotation2d.minus(other: UnboundedRotation) = plus(-other)