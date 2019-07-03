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
import frc.robot.Ports.kPCMID
import frc.robot.Robot
import org.ghrobotics.lib.localization.Localization
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.feet
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
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.MultiMotorTransmission
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.properties.Delegates

object DriveSubsystem : TankDriveSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    override val leftMotor: MultiMotorTransmission<Length> = object : MultiMotorTransmission<Length>(

            unregisterSubsystem = true) {

        override val master = FalconSRX(LEFT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(LEFT_PORTS[1], DefaultNativeUnitModel))

        init {
            outputInverted = true
        }

        override fun setClosedLoopGains() {
            if(lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
        }
    }

    override val rightMotor: MultiMotorTransmission<Length> = object : MultiMotorTransmission<Length>(unregisterSubsystem = true) {

        override val master = FalconSRX(RIGHT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(RIGHT_PORTS[1], DefaultNativeUnitModel))

        override fun setClosedLoopGains() {
            if(lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
        }
    }

    private val ahrs = AHRS(SPI.Port.kMXP)
    override val localization = TankEncoderLocalization(
            ahrs.asSource(),
            {
                leftMotor.currentState.position},
            {rightMotor.currentState.position})


    // init localization stuff
    init {localization.reset(Pose2d()) }
    override fun lateInit() {

        robotPosition = Pose2d(Translation2d(20.feet, 20.feet))

        Notifier {
            localization.update()
//            println("localization updated")
        }.startPeriodic(1.0 / 100.0)
        Robot.subsystemUpdateList.plusAssign(this)

        defaultCommand = ManualDriveCommand() // set default command
    }

    // Ramsete gang is the only true gang
    override var trajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    // the "differential drive" model, with a custom getter which changes based on the current gear
    override val differentialDrive: DifferentialDrive
        get() = if(lowGear) Constants.DriveConstants.kLowGearDifferentialDrive else Constants.DriveConstants.kHighGearDifferentialDrive

    // Shift up and down
    private val shifter = FalconDoubleSolenoid(SHIFTER_PORTS[0], SHIFTER_PORTS[1], kPCMID)
    var lowGear : Boolean by Delegates.observable(false) { _, _, wantLow ->
        if (wantLow) { shifter.state = FalconSolenoid.State.Forward }
        else { shifter.state = FalconSolenoid.State.Reverse }

        // update PID gains
        leftMotor.setClosedLoopGains()
        rightMotor.setClosedLoopGains()
    }

    override fun updateState() {
//        println("updating motor states")
        leftMotor.updateState()
        rightMotor.updateState()
    }

    override fun setNeutral() = run { leftMotor.setNeutral();rightMotor.setNeutral() }

    override fun activateEmergency() = run { zeroOutputs();leftMotor.zeroClosedLoopGains();rightMotor.zeroClosedLoopGains() }

    override fun recoverFromEmergency() = run { leftMotor.setClosedLoopGains();rightMotor.setClosedLoopGains() }

}








