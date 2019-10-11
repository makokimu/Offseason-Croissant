package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import kotlin.math.absoluteValue

class TurnInPlaceCommand(val angle: Rotation2d) : FalconCommand(DriveSubsystem) {

    constructor(angle: SIUnit<Radian>) : this(angle.toRotation2d())

    val prevError = 0.0

    override fun execute() {
        val error = (DriveSubsystem.robotPosition.rotation - angle).radian
        val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)

        println("error $error turn $turn")

        DriveSubsystem.setWheelVelocities(DifferentialDrive.WheelState(-turn, turn))
    }

    override fun isFinished() = (DriveSubsystem.robotPosition.rotation - angle).radian.absoluteValue < 2.degree.radian

    override fun end(interrupted: Boolean) {
        DriveSubsystem.setNeutral()
    }

    companion object {
        const val kCorrectionKp = 0.25
        const val kCorrectionKd = 0.0
    }
}