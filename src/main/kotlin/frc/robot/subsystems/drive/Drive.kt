package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.DoubleSolenoid
import org.ghrobotics.lib.localization.Localization
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.wrappers.FalconMotor
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerDriveBase
import org.ghrobotics.lib.wrappers.LinearFalconMotor

class Drive(
        val left:  FalconMotor<Length>,
        val right: FalconMotor<Length>,
        val shifter: DoubleSolenoid,
        val gyro: AHRS,
        val localization: Localization,
        val leftDistance: Length,
        val rightDistance: Length
            ) : TrajectoryTrackerDriveBase {

    lateinit var trajectoryTracker : TrajectoryTracker

    init {
        trajectoryTracker = RamseteTracker(kBeta, kZeta)
        localization.reset()
        Notifier{
            localization.update((Timer.getFPGATimestamp()).second)
        }.startPeriodic(1.0 / 100.0)
    }

    // Shift up and down
    var lowGear by Delegates.observable(false) { _: Boolean, _ : Boolean, wantLow : Boolean ->
        if (wantLow) {
            shifter.set(kForward)
        } else {
            shifter.set(kReverse)
        }
    }

    override val leftMotor: LinearFalconMotor
        get() = left

    override val rightMotor: LinearFalconMotor
        get() = right

    override val robotLocation: Pose2d
        get() = localization.get(Timer.getFPGATimestamp().second)

    override val trajectoryTracker: TrajectoryTracker
        get() =

}