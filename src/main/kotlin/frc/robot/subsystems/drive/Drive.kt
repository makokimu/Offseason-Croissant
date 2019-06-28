package frc.robot.subsystems.drive

import asSource
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.kauailabs.navx.frc.AHRS
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.Length
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.experimental.command.CommandScheduler
import frc.robot.Constants
import frc.robot.Constants.DriveConstants.kDriveLengthModel
import frc.robot.Ports
import frc.robot.Ports.DrivePorts.LEFT_PORTS
import frc.robot.Ports.DrivePorts.RIGHT_PORTS
import frc.robot.Ports.DrivePorts.SHIFTER_PORTS
import org.ghrobotics.lib.localization.Localization
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.motors.FalconEncoder
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.LinearFalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.ctre.FalconCTREEncoder
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.team5940.pantry.lib.MultiMotorTransmission
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.properties.Delegates

object Drive : TankDriveSubsystem(), EmergencyHandleable {

    override val leftMotor: MultiMotorTransmission<Length> = object : MultiMotorTransmission<Length>(
            unregisterSubsystem = true) {

        override val master = FalconSRX(LEFT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(LEFT_PORTS[1], DefaultNativeUnitModel))

        override fun setClosedLoopGains() {
            if(lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
        }
    }

    override val rightMotor: MultiMotorTransmission<Length> = object : MultiMotorTransmission<Length>(
            unregisterSubsystem = true) {

        override val master = FalconSRX(RIGHT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(RIGHT_PORTS[1], DefaultNativeUnitModel))

        override fun setClosedLoopGains() {
            if(lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
        }
    }

    private val ahrs = AHRS(SPI.Port.kMXP)
    override val localization = TankEncoderLocalization(
            ahrs.asSource(),
            {leftMotor.encoder.position},
            {rightMotor.encoder.position})

    override var trajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    override val differentialDrive: DifferentialDrive
        get() = if(lowGear) Constants.DriveConstants.kLowGearDifferentialDrive else Constants.DriveConstants.kHighGearDifferentialDrive

    // Shift up and down
    val shifter = FalconDoubleSolenoid(SHIFTER_PORTS[0], SHIFTER_PORTS[1])
    var lowGear : Boolean by Delegates.observable(false) { _, _, wantLow ->
        if (wantLow) {
            shifter.state = FalconSolenoid.State.Forward
        } else {
            shifter.state = FalconSolenoid.State.Reverse
        }

        // update PID gains
        leftMotor.setClosedLoopGains()
        rightMotor.setClosedLoopGains()
    }

    init {localization.reset(Pose2d()) }
    override fun lateInit() { Notifier(localization::update).startPeriodic(1.0 / 100.0) }
    override fun setNeutral() {
        leftMotor.setNeutral()
        rightMotor.setNeutral()
    }

    override fun activateEmergency() {
        zeroOutputs()
        leftMotor.zeroClosedLoopGains()
        rightMotor.zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() {
        leftMotor.setClosedLoopGains()
        rightMotor.setClosedLoopGains()
    }

}








