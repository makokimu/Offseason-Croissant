package frc.robot.subsystems.drive

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.kauailabs.navx.frc.AHRS
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.DoubleSolenoid
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.Length
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.SPI
import frc.robot.Constants
import frc.robot.Ports
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.motors.FalconEncoder
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.LinearFalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTREEncoder
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.drive.DifferentialTrackerDriveBase
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.team5940.pantry.exparimental.command.RunCommand
import org.team5940.pantry.exparimental.command.SendableSubsystemBase
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.properties.Delegates

class Drive(
        override val leftMotor: FalconMotor<Length>,
        override val rightMotor: FalconMotor<Length>,
        val leftEncoder: FalconEncoder<Length>,
        val rightEncoder: FalconEncoder<Length>,
        val shifter: FalconSolenoid,
        val gyro: AHRS,
        val localization: TankEncoderLocalization
            ) : DifferentialTrackerDriveBase, SendableSubsystemBase() {

    var currentTrajectoryTracker : TrajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    override val robotPosition by localization

    init {
//        trajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)
        localization.reset(Pose2d())
        Notifier(localization::update).startPeriodic(1.0 / 100.0)
    }

    inner class curvatureDriveCommand(left : DoubleSource, right : DoubleSource, isQuickTurn: BooleanSource) : RunCommand(
            Runnable {
        val commandedInput = curvatureDrive(left.invoke(), right.invoke(), isQuickTurn.invoke())
        leftMotor.setDutyCycle(commandedInput.left)
        rightMotor.setDutyCycle(commandedInput.right)
    }, this)

    // Shift up and down
    var lowGear : Boolean by Delegates.observable(false) { _, _, wantLow ->
        if (wantLow) {
            shifter.state = FalconSolenoid.State.Forward
        } else {
            shifter.state = FalconSolenoid.State.Reverse
        }
    }

    override val trajectoryTracker: TrajectoryTracker
        get() = currentTrajectoryTracker

    override val differentialDrive: DifferentialDrive
        get() = if(lowGear) Constants.DriveConstants.kLowGearDifferentialDrive else Constants.DriveConstants.kHighGearDifferentialDrive


    private var quickStopAccumulator = 0.0

    /**
     * Curvature or cheezy drive control
     */
    @Suppress("ComplexMethod")
    private fun curvatureDrive (
            linearPercent: Double,
            curvaturePercent: Double,
            isQuickTurn: Boolean
    ) : DifferentialDrive.WheelState {
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

        return DifferentialDrive.WheelState(leftMotorOutput, rightMotorOutput)
    }

    companion object {

        const val kQuickStopThreshold = edu.wpi.first.wpilibj.drive.DifferentialDrive.kDefaultQuickStopThreshold
        const val kQuickStopAlpha = edu.wpi.first.wpilibj.drive.DifferentialDrive.kDefaultQuickStopAlpha

        fun getRealTalonDrive() : Drive {
            val leftMotors = listOf(
                    FalconSRX(Ports.DrivePorts.LEFT_PORTS[0], Constants.DriveConstants.kDriveLengthModel),
                    FalconSRX(Ports.DrivePorts.LEFT_PORTS[1], Constants.DriveConstants.kDriveLengthModel)
            )

            val rightMotors = listOf(
                    FalconSRX(Ports.DrivePorts.RIGHT_PORTS[0], Constants.DriveConstants.kDriveLengthModel),
                    FalconSRX(Ports.DrivePorts.RIGHT_PORTS[1], Constants.DriveConstants.kDriveLengthModel)
            )

            leftMotors.forEach{
                it.outputInverted = true
            }

            leftMotors[0].feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative
            rightMotors[0].feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative

            val leftEncoder = FalconCTREEncoder(leftMotors[0].motorController, model=leftMotors[0].model)
            val rightEncoder = FalconCTREEncoder(rightMotors[0].motorController, model=rightMotors[0].model)

            val leftTransmission = Transmission(leftMotors)
            val rightTransmission = Transmission(rightMotors)

            val shifter = FalconDoubleSolenoid(Ports.DrivePorts.SHIFTER_PORTS[0], Ports.DrivePorts.SHIFTER_PORTS[1], Ports.kPCMID)

            val gyro = AHRS(SPI.Port.kMXP)

            val localization = TankEncoderLocalization(
                    {(gyro.getFusedHeading() * -1).degree},
                    {leftEncoder.position},
                    {rightEncoder.position}
            )

            return Drive(
                    leftTransmission,
                    rightTransmission,
                    leftEncoder,
                    rightEncoder,
                    shifter,
                    gyro,
                    localization
            )

        }
    }

}








