package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Constants
import frc.robot.Constants.DriveConstants.kDriveLengthModel
import frc.robot.Ports.DrivePorts.LEFT_PORTS
import frc.robot.Ports.DrivePorts.RIGHT_PORTS
import frc.robot.Ports.DrivePorts.SHIFTER_PORTS
import frc.robot.Ports.kPCMID
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.asSource
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.MultiMotorTransmission
import kotlin.properties.Delegates

object DriveSubsystem : FalconWestCoastDrivetrain(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    override val leftMotor: MultiMotorTransmission<Meter, FalconSRX<Meter>> = object : MultiMotorTransmission<Meter, FalconSRX<Meter>>() {

        override val master = FalconSRX(LEFT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(LEFT_PORTS[1], DefaultNativeUnitModel))

        init {
            outputInverted = true
            followers.forEach { it.follow(master) }
            lateInit()
        }

        override fun setClosedLoopGains() {
            // LQR gains
            if (lowGear) setClosedLoopGains(0.667, 0.0) else setClosedLoopGains(0.92, 0.0)
            // old gains
//            if (lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
        }
    }

    override val rightMotor: MultiMotorTransmission<Meter, FalconSRX<Meter>> = object : MultiMotorTransmission<Meter, FalconSRX<Meter>>() {

        override val master = FalconSRX(RIGHT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(RIGHT_PORTS[1], DefaultNativeUnitModel))

        init {
            followers.forEach { it.follow(master) }
            lateInit()
        }

        override fun setClosedLoopGains() {
            // LQR gains
            if (lowGear) setClosedLoopGains(0.667, 0.0) else setClosedLoopGains(0.92, 0.0)
            // Old gains
//            if (lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
        }
    }

    override val leftCharacterization: MotorCharacterization<Meter>
        get() = if(lowGear) Constants.DriveConstants.kLeftCharacterizationLow else Constants.DriveConstants.kLeftCharacterizationHigh


    override val rightCharacterization: MotorCharacterization<Meter>
        get() = if(lowGear) Constants.DriveConstants.kRightCharacterizationLow else Constants.DriveConstants.kRightCharacterizationHigh

    override fun setNeutral() {
        leftMotor.setNeutral(); rightMotor.setNeutral()
        super.setNeutral()
    }

    override fun activateEmergency() = run { setNeutral(); leftMotor.zeroClosedLoopGains(); rightMotor.zeroClosedLoopGains() }

    override fun recoverFromEmergency() = run { leftMotor.setClosedLoopGains(); rightMotor.setClosedLoopGains() }
    fun notWithinRegion(region: Rectangle2d): CommandBase =
            WaitUntilCommand { !region.contains(robotPosition.translation) }

    // Shift up and down
    private val shifter = FalconDoubleSolenoid(SHIFTER_PORTS[0], SHIFTER_PORTS[1], kPCMID)
    var lowGear: Boolean by Delegates.observable(false) { _, _, wantsLow ->

        shifter.state = if (wantsLow) FalconSolenoid.State.Reverse else FalconSolenoid.State.Forward

        // update PID gains
        leftMotor.setClosedLoopGains()
        rightMotor.setClosedLoopGains()
    }

    // the "differential drive" model, with a custom getter which changes based on the current gear
    override val kinematics = DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth.inMeters())

    private val ahrs = AHRS(SPI.Port.kMXP)
    override val odometry = DifferentialDriveOdometry(kinematics)
//            ahrs.asSource(),
//            { leftMotor.encoder.position },
//            { rightMotor.encoder.position })

    // init localization stuff
    override fun lateInit() {
        // set the robot pose to a sane position
        robotPosition = Pose2d(20.feet, 20.feet, 0.degrees.toRotation2d())
//        defaultCommand = ManualDriveCommand() // set default command
        defaultCommand = ClosedLoopChezyDriveCommand()
        super.lateInit()
    }

    // Ramsete gang is the only true gang
    override var controller = RamseteController(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    fun setWheelVelocities(wheelSpeeds: DifferentialDriveWheelSpeeds) {
        val leftFF = leftCharacterization.getVoltage(wheelSpeeds.leftMetersPerSecond.meters.velocity, 0.meters.acceleration)
        val rightFF = leftCharacterization.getVoltage(wheelSpeeds.rightMetersPerSecond.meters.velocity, 0.meters.acceleration)

        leftMotor.setVelocity(wheelSpeeds.leftMetersPerSecond.meters.velocity, leftFF)
        rightMotor.setVelocity(wheelSpeeds.rightMetersPerSecond.meters.velocity, rightFF)
    }
}
