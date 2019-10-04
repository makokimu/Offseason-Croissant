package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.kFeetToMeter
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.pow

class ClosedLoopChezyDriveCommand: ManualDriveCommand() {

    override fun initialize() {
        super.initialize()
        DriveSubsystem.leftMotor.master.talonSRX.configClosedloopRamp(0.12)
        DriveSubsystem.rightMotor.master.talonSRX.configClosedloopRamp(0.12)
    }

    override fun execute() {
        val curvature = rotationSource()
        val linear = -speedSource()
        val isQuickTurn = quickTurnSource()
        val multiplier = if(DriveSubsystem.lowGear) 8.0 * kFeetToMeter else 11.0 * kFeetToMeter
        var wheelSpeeds = curvatureDrive(linear, curvature, isQuickTurn)
        wheelSpeeds = DifferentialDrive.WheelState(wheelSpeeds.left * multiplier, wheelSpeeds.right * multiplier)
        DriveSubsystem.setWheelVelocities(wheelSpeeds)
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        DriveSubsystem.leftMotor.master.talonSRX.configClosedloopRamp(0.0)
        DriveSubsystem.rightMotor.master.talonSRX.configClosedloopRamp(0.0)
    }

}

operator fun DifferentialDrive.WheelState.times(scaler: Double) = DifferentialDrive.WheelState(left * scaler, right * scaler)