package frc.robot.subsystems.drive

import asSource
import com.kauailabs.navx.frc.AHRS
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import edu.wpi.first.wpilibj.frc2.command.WaitUntilCommand
import frc.robot.Constants
import frc.robot.Constants.DriveConstants.kDriveLengthModel
import frc.robot.Ports.DrivePorts.LEFT_PORTS
import frc.robot.Ports.DrivePorts.RIGHT_PORTS
import frc.robot.Ports.DrivePorts.SHIFTER_PORTS
import frc.robot.Ports.kPCMID
import io.github.oblarg.oblog.Loggable
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.MultiMotorTransmission
import kotlin.properties.Delegates

object DriveSubsystem : TankDriveSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent, Loggable {

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

    override fun setNeutral() {
        leftMotor.setNeutral(); rightMotor.setNeutral()
        super.setNeutral()
    }

    override fun activateEmergency() = run { zeroOutputs(); leftMotor.zeroClosedLoopGains(); rightMotor.zeroClosedLoopGains() }

    override fun recoverFromEmergency() = run { leftMotor.setClosedLoopGains(); rightMotor.setClosedLoopGains() }
    fun notWithinRegion(region: Rectangle2d) =
            WaitUntilCommand { !region.contains(robotPosition.translation) }

    // Shift up and down
    private val shifter = FalconDoubleSolenoid(SHIFTER_PORTS[0], SHIFTER_PORTS[1], kPCMID)
    var lowGear: Boolean by Delegates.observable(false) { _, _, wantsLow ->

        shifter.state = if (wantsLow) FalconSolenoid.State.Reverse else FalconSolenoid.State.Forward

        // update PID gains
        leftMotor.setClosedLoopGains()
        rightMotor.setClosedLoopGains()
    }
    class SetGearCommand(wantsLow: Boolean) : InstantCommand(Runnable { lowGear = wantsLow }, this)

    private val ahrs = AHRS(SPI.Port.kMXP)
    override val localization = TankEncoderLocalization(
            ahrs.asSource(),
            { leftMotor.encoder.position },
            { rightMotor.encoder.position })

    // init localization stuff
    override fun lateInit() {
        // set the robot pose to a sane position
        robotPosition = Pose2d(translation = Translation2d(20.feet, 20.feet), rotation = 0.degree)
//        defaultCommand = ManualDriveCommand() // set default command
        defaultCommand = ClosedLoopChezyDriveCommand()
        super.lateInit()
    }

    // Ramsete gang is the only true gang
    override var trajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    // the "differential drive" model, with a custom getter which changes based on the current gear
    override val differentialDrive: DifferentialDrive
        get() = if (lowGear) Constants.DriveConstants.kLowGearDifferentialDrive else Constants.DriveConstants.kHighGearDifferentialDrive

    override fun updateState() {
//        localization.update()
    }

    /**
     * Returns the follow trajectory command
     *
     * @param trajectory Source with the trajectory to follow
     * @param pathMirrored Whether to mirror the path or not
     */
    fun driveTrajectory(
            trajectory: Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>,
            pathMirrored: BooleanSource
    ) = TrajectoryTrackerCommand(pathMirrored.map(trajectory.mirror(), trajectory))

    fun setWheelVelocities(wheelSpeeds: DifferentialDrive.WheelState) {
        val left = wheelSpeeds.left / differentialDrive.wheelRadius // rad per sec
        val right = wheelSpeeds.right / differentialDrive.wheelRadius // rad per sec
        val ff = differentialDrive.getVoltagesFromkV(DifferentialDrive.WheelState(left, right))
        leftMotor.setVelocity(wheelSpeeds.left.meter.velocity, ff.left.volt)
        rightMotor.setVelocity(wheelSpeeds.right.meter.velocity, ff.right.volt)
    }
}
