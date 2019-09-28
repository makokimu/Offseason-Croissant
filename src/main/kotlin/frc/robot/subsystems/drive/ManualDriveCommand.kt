package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.pow

open class ManualDriveCommand : FalconCommand(DriveSubsystem) {

    override fun execute() {
        val curvature = rotationSource()
        val linear = -speedSource()
        val driveCubicDeadband = ((cubicPrecision * (linear pow 3) + (1.0 - cubicPrecision) * curvature) - (abs(curvature) / curvature) * (cubicPrecision * (kDeadband pow 3) + (1.0 - cubicPrecision) * kDeadband)) / (1.0 - (cubicPrecision * (kDeadband pow 3) + (1.0 - cubicPrecision) * kDeadband))

//        println("Drive motor power $linear")

        DriveSubsystem.curvatureDrive(
                linear,
                curvature,
                quickTurnSource())

//        curvatureDrive(
//                linear,
//                driveCubicDeadband,
//                quickTurnSource())
    }

    /**
     * Curvature or cheezy drive control
     */
    @Suppress("ComplexMethod")
    private fun curvatureDrive(
        linearPercent: Double,
        curvaturePercent: Double,
        isQuickTurn: Boolean
    ) {
        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (linearPercent.absoluteValue < TankDriveSubsystem.kQuickStopThreshold) {
                quickStopAccumulator = (1 - TankDriveSubsystem.kQuickStopAlpha) * quickStopAccumulator +
                        TankDriveSubsystem.kQuickStopAlpha * curvaturePercent.coerceIn(-1.0, 1.0) * 2.0
            }
            overPower = true
            angularPower = curvaturePercent
        } else {
            overPower = false
            angularPower = linearPercent.absoluteValue * curvaturePercent - quickStopAccumulator

            when {
                quickStopAccumulator > 1 -> quickStopAccumulator -= 1.0
                quickStopAccumulator < -1 -> quickStopAccumulator += 1.0
                else -> quickStopAccumulator = 0.0
            }
        }

        var leftMotorOutput = linearPercent + angularPower
        var rightMotorOutput = linearPercent - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            when {
                leftMotorOutput > 1.0 -> {
                    rightMotorOutput -= leftMotorOutput - 1.0
                    leftMotorOutput = 1.0
                }
                rightMotorOutput > 1.0 -> {
                    leftMotorOutput -= rightMotorOutput - 1.0
                    rightMotorOutput = 1.0
                }
                leftMotorOutput < -1.0 -> {
                    rightMotorOutput -= leftMotorOutput + 1.0
                    leftMotorOutput = -1.0
                }
                rightMotorOutput < -1.0 -> {
                    leftMotorOutput -= rightMotorOutput + 1.0
                    rightMotorOutput = -1.0
                }
            }
        }

        // Normalize the wheel speeds
        val maxMagnitude = max(leftMotorOutput.absoluteValue, rightMotorOutput.absoluteValue)
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude
            rightMotorOutput /= maxMagnitude
        }

        print(" left motor $leftMotorOutput right $rightMotorOutput")

        tankDrive(leftMotorOutput, rightMotorOutput)
    }

    /**
     * Tank drive control
     */
    private fun tankDrive(
        leftPercent: Double,
        rightPercent: Double
    ) {
        DriveSubsystem.leftMotor.setDutyCycle(leftPercent)
        DriveSubsystem.rightMotor.setDutyCycle(rightPercent)
    }

    companion object {
        private var kLockVelocityTolerance = 0.5.feet.velocity
        private var quickStopAccumulator = 0.0
        private const val kQuickStopThreshold = TankDriveSubsystem.kQuickStopThreshold
        private const val kQuickStopAlpha = TankDriveSubsystem.kQuickStopAlpha
        const val kDeadband = 0.05
        private const val cubicPrecision = 0.1
        val speedSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft) }
        private val rotationSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kRight) }
        private val quickTurnSource by lazy { Controls.driverFalconXbox.getRawButton(kBumperRight)/*.getRawButton(kX)8*/ }
    }
}

private infix fun Number.pow(i: Number): Double = toDouble().pow(i.toDouble())
