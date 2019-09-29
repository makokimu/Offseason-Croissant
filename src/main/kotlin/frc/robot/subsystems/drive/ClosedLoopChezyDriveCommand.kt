package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.kFeetToMeter
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.pow

class ClosedLoopChezyDriveCommand: ManualDriveCommand() {

    override fun execute() {
        val curvature = rotationSource()
        val linear = -speedSource()
        val isQuickTurn = quickTurnSource()
        val wheelSpeeds = curvatureDrive(curvature, linear, isQuickTurn).times(
                if(DriveSubsystem.lowGear) 8.0 * kFeetToMeter else 12.0 * kFeetToMeter
        )
        val feedForwards = DriveSubsystem.differentialDrive.getVoltagesFromkV(wheelSpeeds)
        DriveSubsystem.setOutput(wheelSpeeds, feedForwards)
    }

}

operator fun DifferentialDrive.WheelState.times(scaler: Double) = DifferentialDrive.WheelState(left * scaler, right * scaler)