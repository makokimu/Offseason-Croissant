package frc.robot.subsystems.drive

import asSource
import com.kauailabs.navx.frc.AHRS
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.units.Length
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.experimental.command.WaitUntilCommand
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
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.MultiMotorTransmission
import kotlin.properties.Delegates

object DriveSubsystem : TankDriveSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent, Loggable {

    override val leftMotor: MultiMotorTransmission<Length, FalconSRX<Length>> = object : MultiMotorTransmission<Length, FalconSRX<Length>>() {

        override val master = FalconSRX(LEFT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(LEFT_PORTS[1], DefaultNativeUnitModel))

        init {
            outputInverted = true
        }

        override fun setClosedLoopGains() {
            if (lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
        }
    }

    override val rightMotor: MultiMotorTransmission<Length, FalconSRX<Length>> = object : MultiMotorTransmission<Length, FalconSRX<Length>>() {

        override val master = FalconSRX(RIGHT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(RIGHT_PORTS[1], DefaultNativeUnitModel))

        override fun setClosedLoopGains() {
            if (lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.2, 10.0)
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

        shifter.state = if(wantsLow) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse

        // update PID gains
        leftMotor.setClosedLoopGains()
        rightMotor.setClosedLoopGains()
    }

    private val ahrs = AHRS(SPI.Port.kMXP)
    override val localization = TankEncoderLocalization(
            ahrs.asSource(),
            { leftMotor.encoder.position },
            { rightMotor.encoder.position })

    // init localization stuff
    override fun lateInit() {
        // set the robot pose to a sane position
        robotPosition = Pose2d(translation = Translation2d(20.feet, 20.feet), rotation = UnboundedRotation.kZero)
        defaultCommand = ManualDriveCommand() // set default command
        super.lateInit()
    }

    // Ramsete gang is the only true gang
    override var trajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    // the "differential drive" model, with a custom getter which changes based on the current gear
    override val differentialDrive: DifferentialDrive
        get() = if (lowGear) Constants.DriveConstants.kLowGearDifferentialDrive else Constants.DriveConstants.kHighGearDifferentialDrive

    override suspend fun updateState() {
//        localization.update()
    }
}
