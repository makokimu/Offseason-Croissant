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
        println("linear $linear curve $curvature")
        val multiplier = if(DriveSubsystem.lowGear) 8.0 * kFeetToMeter else 12.0 * kFeetToMeter
        var wheelSpeeds = curvatureDrive(curvature, linear, isQuickTurn)
        wheelSpeeds = DifferentialDrive.WheelState(wheelSpeeds.left * multiplier, wheelSpeeds.right * multiplier)
        val feedForwards = DriveSubsystem.differentialDrive.getVoltagesFromkV(wheelSpeeds)
        println("speeds $wheelSpeeds ff $feedForwards")
        DriveSubsystem.setOutput(wheelSpeeds, feedForwards)
    }

}

operator fun DifferentialDrive.WheelState.times(scaler: Double) = DifferentialDrive.WheelState(left * scaler, right * scaler)