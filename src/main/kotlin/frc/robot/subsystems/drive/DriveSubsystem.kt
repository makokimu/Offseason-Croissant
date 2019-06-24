package frc.robot.subsystems.drive

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.kauailabs.navx.frc.AHRS
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.SPI
import org.ghrobotics.lib.components.DriveComponent
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.motors.LinearFalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.sensors.asSource
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import kotlin.properties.Delegates

object DriveSubsystem : DriveComponent(2.inch) {

    val lowGearDiffDrive = DriveConstants.kLowGearDifferentialDrive
    val highGearDiffDrive = DriveConstants.kHighGearDifferentialDrive

    val shifter = FalconDoubleSolenoid(4,5, 9)

    // Shift up and down
    var lowGear : Boolean by Delegates.observable(false) { _, _, wantLow ->
        if (wantLow) {
            shifter.state = FalconSolenoid.State.Forward
        } else {
            shifter.state = FalconSolenoid.State.Reverse
        }
    }

    override val differentialDrive: DifferentialDrive
        get() = (if(lowGear) lowGearDiffDrive else highGearDiffDrive)

    private val leftSRX = TalonSRX(1)
    private val rightSRX = TalonSRX(3)
    override val leftMotor = FalconSRX(leftSRX, NativeUnitLengthModel(4096.nativeUnits, 4.inch))
    override val rightMotor = FalconSRX(rightSRX, NativeUnitLengthModel(4096.nativeUnits, 4.inch))

    override val trajectoryTracker = RamseteTracker(2.0, 0.7)

    val gyro = AHRS(SPI.Port.kMXP)

    val localization = TankEncoderLocalization(
            gyro.asSource(),
            {leftMotor.encoder.position},
            {rightMotor.encoder.position}
    )

    override val robotPosition = localization.robotPosition

}