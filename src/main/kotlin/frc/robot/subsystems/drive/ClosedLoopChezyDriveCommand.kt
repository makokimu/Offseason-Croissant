package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.kFeetToMeter
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.pow

class ClosedLoopChezyDriveCommand: ManualDriveCommand() {

    companion object {
        val kMaxLinearAcceleration = 10.0 * kFeetToMeter
        var lastLinearVelocity = 0.0
    }

    override fun initialize() {
        super.initialize()
        DriveSubsystem.leftMotor.master.talonSRX.configClosedloopRamp(0.12)
        DriveSubsystem.rightMotor.master.talonSRX.configClosedloopRamp(0.12)
    }

    override fun execute() {
        val curvature = rotationSource()
        var linear = -speedSource()
        val isQuickTurn = quickTurnSource()

        // limit linear acceleration
        if(lastLinearVelocity + kMaxLinearAcceleration * 0.020 < linear) linear = lastLinearVelocity + kMaxLinearAcceleration * 0.020
        if(lastLinearVelocity - kMaxLinearAcceleration * 0.020 > linear) linear = lastLinearVelocity - kMaxLinearAcceleration * 0.020

        val multiplier = if(DriveSubsystem.lowGear) 8.0 * kFeetToMeter else 11.0 * kFeetToMeter
        var wheelSpeeds = curvatureDrive(linear, curvature, isQuickTurn)
        wheelSpeeds = DifferentialDrive.WheelState(wheelSpeeds.left * multiplier, wheelSpeeds.right * multiplier)
        DriveSubsystem.setWheelVelocities(wheelSpeeds)

        lastLinearVelocity = linear
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        DriveSubsystem.leftMotor.master.talonSRX.configClosedloopRamp(0.0)
        DriveSubsystem.rightMotor.master.talonSRX.configClosedloopRamp(0.0)
    }

}

operator fun DifferentialDrive.WheelState.times(scaler: Double) = DifferentialDrive.WheelState(left * scaler, right * scaler)