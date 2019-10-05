package frc.robot

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.physics.MotorCharacterization
import kotlin.math.PI

object Constants {

    const val kIsRocketLeague =
            true

    object DriveConstants {
        const val kBeta = 2.0
        const val kZeta = 0.7

        val kRobotMass = (50.0 /* Robot, kg */ + 5.0 /* Battery, kg */ + 2.0 /* Bumpers, kg */).toDouble()
        private val kRobotMomentOfInertia = 10.0 // kg m^2 // TODO Tune
        private val kRobotAngularDrag = 12.0 // N*m / (rad/sec)

        val kWheelRadius = (2.0).inches
        val kTrackWidth = (26.0).inches

        val kDriveLengthModel = NativeUnitLengthModel(4096.nativeUnits, kWheelRadius)

        private val kVDriveLeftLow = 0.274 // Volts per radians per second
        private val kADriveLeftLow = 0.032 // Volts per radians per second per second
        private val kVInterceptLeftLow = 1.05 // Volts

        private val kVDriveRightLow = 0.265 // Volts per radians per second
        private val kADriveRightLow = 0.031 // Volts per radians per second per second
        private val kVInterceptRightLow = 1.02 // Volts

        val kLeftCharacterizationLow = MotorCharacterization<Meter>(
                // volts per radian per second times radians per meter
                // = volts per rad per second div meters per radian
                // = volts per rad per second div circumference per 2pi radians
                // = volts per rad per second div 2 pi radius per 2 pi radinas
                // = volts per rad per second div radius
                SIUnit(kVDriveLeftLow / kWheelRadius.inMeters()),
                SIUnit(kADriveLeftLow / kWheelRadius.inMeters()),
                kVInterceptLeftLow.volts
        )

        val kRightCharacterizationLow = MotorCharacterization<Meter>(
                SIUnit(kVDriveRightLow / kWheelRadius.inMeters()),
                SIUnit(kADriveRightLow / kWheelRadius.inMeters()),
                kVInterceptRightLow.volts
        )

        private val kVDriveLeftHigh = 0.143 * 1.0 // Volts per radians per second
        private val kADriveLeftHigh = 0.043 * 1.0 // Volts per radians per second per second
        private val kVInterceptLeftHigh = 1.33 * 1.0 // 4 * 0.4d; // Volts

        private val kVDriveRightHigh = 0.14 * 1.0 // Volts per radians per second
        private val kADriveRightHigh = 0.043 * 1.0 // Volts per radians per second per second
        private val kVInterceptRightHigh = 1.34 * 1.0 // 4 * 0.4d; // Volts

        val kLeftCharacterizationHigh = MotorCharacterization<Meter>(
                // volts per radian per second times radians per meter
                // = volts per rad per second div meters per radian
                // = volts per rad per second div circumference per 2pi radians
                // = volts per rad per second div 2 pi radius per 2 pi radinas
                // = volts per rad per second div radius
                SIUnit(kVDriveLeftHigh / kWheelRadius.inMeters()),
                SIUnit(kADriveLeftHigh / kWheelRadius.inMeters()),
                kVInterceptLeftHigh.volts
        )

        val kRightCharacterizationHigh = MotorCharacterization<Meter>(
                SIUnit(kVDriveRightHigh / kWheelRadius.inMeters()),
                SIUnit(kADriveRightHigh / kWheelRadius.inMeters()),
                kVInterceptRightHigh.volts
        )

    }

    object SuperStructureConstants {
        val kProximalStatic = 0.4.volts // volts
        val kProximalCos = 0.94.volts // volts
        const val kJointSpeedMultiplier = 0.95
        val kProximalLen = 32.0.inches
        val kElevatorRange = 11.inches..69.inches
        val kProximalThrustVelocity = 50.degrees.velocity
    }

    object IntakeConstants {

//        val deployTime = 0.1.second
    }

    /* Wrist stuff */
    val kWristLength = 6.inches // distance from joint to COM
    val kWristMass = 15.lb
    val kWristSpeedPerVolt = 0.21 // radians/sec/volt
    val kWristTorquePerVolt = 47.33 // Newton meters per volt, stall
    val kWristStaticFrictionVoltage = 0.0 // volts, TODO tune

    /* Elbow stuff */
    val kElbowLength = 8.inches // distance from joint to COM
    val kElbowMass = 3.lb
    val kElbowSpeedPerVolt = 0.17 // radians/sec/volt
    val kElbowTorquePerVolt = 55.0 // Newton meters per volt, stall
    val kElbowStaticFrictionVoltage = 0.0 // volts, TODO tune

    // ROBOT AND MECHANISM DIMENSIONS

    val kRobotWidth = 28.75.inches
    val kRobotLength = 31.inches

    val kBumperThickness = 3.25.inches
    val kCenterToElevator = (kRobotLength / 2) - 11.inches // 4.5
    val kBadIntakeOffset = 0.inches
    val kArmLength = 29.5.inches // from center of elevator to hatch part of the intake

    val kIntakeProtrusionFrontExtended = kArmLength - (kRobotLength / 2.0 - kCenterToElevator) // 18.5
    val kIntakeProtrusionBackExtended = kCenterToElevator - kArmLength + kRobotLength / 2.0 // -9.5

    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meters, 0.degrees)
    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness, 0.meters, 0.degrees)

    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusionFrontExtended, kBadIntakeOffset, 0.degrees) // -34
    val kCenterToForwardIntake = Pose2d((kRobotLength / 2.0) + kIntakeProtrusionFrontExtended, -kBadIntakeOffset, 0.degrees) // 34
    val kBackwardIntakeToCenter = Pose2d(kCenterToForwardIntake.translation.x.meters - kCenterToElevator, -kBadIntakeOffset, 0.degrees) // 29.5

    val kCenterToFrontCamera = Pose2d(kRobotLength / 2 - 13.inches, 0.0.inches, 0.degrees)
    val kCenterToBackCamera = Pose2d(kRobotLength / 2 - 16.inches, 0.0.inches, 180.degrees)
}

fun Pose2d(x: SIUnit<Meter>, y: SIUnit<Meter>, angle: SIUnit<Radian>) =
        edu.wpi.first.wpilibj.geometry.Pose2d(x.value, y.value, angle.toRotation2d())