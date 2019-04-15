package frc.robot

import com.team254.lib.physics.DifferentialDrive
import com.team254.lib.physics.DCMotorTransmission
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits


object Constants {

    object DriveConstants {
        const val kBeta = 2.0
        const val kZeta = 0.7


        private val kRobotMass = (50.0 /* Robot, kg */ + 5.0 /* Battery, kg */ + 2.0 /* Bumpers, kg */).toDouble()
        private val kRobotMomentOfInertia = 10.0 // kg m^2 // TODO Tune
        private val kRobotAngularDrag = 12.0 // N*m / (rad/sec)

        private val kWheelRadius = (2.0).inch
        private val kTrackWidth = (26.0).inch

        val kDriveLengthModel = NativeUnitLengthModel(4096.nativeUnits, kWheelRadius)

        private val kVDriveLeftLow = 0.274 * 1.0 // Volts per radians per second - Calculated emperically
        private val kADriveLeftLow = 0.032 * 1.0 // Volts per radians per second per second TODO tune
        private val kVInterceptLeftLow = 1.05 * 1.0 // Volts - tuned!

        private val kVDriveRightLow = 0.265 * 1.0 // Volts per radians per second - Calculated emperically
        private val kADriveRightLow = 0.031 * 1.0 // Volts per radians per second per second TODO tune
        private val kVInterceptRightLow = 1.02 * 1.0 // Volts - tuned!

        private val kLeftTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveLeftLow,
                kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveLeftLow),
                kVInterceptLeftLow)

        private val kRightTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveRightLow,
                kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveRightLow),
                kVInterceptRightLow)

        private val kVDriveLeftHigh = 0.143 * 1.0 // Volts per radians per second - Calculated emperically
        private val kADriveLeftHigh = 0.043 * 1.0 // Volts per radians per second per second
        private val kVInterceptLeftHigh = 1.33 * 1.0//4 * 0.4d; // Volts - tuned!

        private val kVDriveRightHigh = 0.14 * 1.0 // Volts per radians per second - Calculated emperically
        private val kADriveRightHigh = 0.043 * 1.0 // Volts per radians per second per second
        private val kVInterceptRightHigh = 1.34 * 1.0//4 * 0.4d; // Volts - tuned!

        private val kLeftTransmissionModelHighGear = DCMotorTransmission(1 / kVDriveLeftHigh,
                kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveLeftHigh),
                kVInterceptLeftHigh)

        private val kRightTransmissionModelHighGear = DCMotorTransmission(1 / kVDriveRightHigh,
                kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveRightHigh),
                kVInterceptRightHigh)

        val kLowGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
                kRobotAngularDrag, kWheelRadius.meter, kTrackWidth.meter / 2.0, kLeftTransmissionModelLowGear, kRightTransmissionModelLowGear)

        val kHighGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
                kRobotAngularDrag, kWheelRadius.meter, kTrackWidth.meter / 2.0, kLeftTransmissionModelHighGear, kRightTransmissionModelHighGear)

    }

}