package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source
import kotlin.math.absoluteValue

class TurnInPlaceCommand(val angle: Source<Rotation2d>) : FalconCommand(DriveSubsystem) {

    constructor(angle: SIUnit<Radian>) : this({angle.toRotation2d()})

    private var angularVelocity = 0.radian.velocity
    private var prevError = 0.0

    private var wantedAngle = Rotation2d()

    override fun initialize() {
        wantedAngle = angle()
        SmartDashboard.putData(this)
    }

    var isOnTarget = false

    override fun execute() {
        val error = (DriveSubsystem.robotPosition.rotation - wantedAngle).radian
        val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
        angularVelocity = ((prevError - error) / 0.020).radian.velocity

        DriveSubsystem.setWheelVelocities(DifferentialDrive.WheelState(turn, -turn))

        prevError = error
    }

    override fun isFinished() = (DriveSubsystem.robotPosition.rotation.radian - wantedAngle.radian).absoluteValue < 4.degree.radian
            && (angularVelocity.absoluteValue.value < 5.degree.radian)

    override fun end(interrupted: Boolean) {
        DriveSubsystem.setNeutral()
    }

//    override fun initSendable(builder: SendableBuilder) {
//        builder.addDoubleProperty("kp", { kCorrectionKp }, {_new -> kCorrectionKp = _new})
//        builder.addDoubleProperty("kd", { kCorrectionKd }, {_new -> kCorrectionKd = _new})
//        super.initSendable(builder)
//    }

    companion object {
        var kCorrectionKp = 1.75
        var kCorrectionKd = 15.0
    }
}