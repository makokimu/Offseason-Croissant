package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration
import com.revrobotics.CANDigitalInput
import frc.robot.lib.MotorHelpers
import frc.robot.lib.MultiMotorTransmission
import org.ghrobotics.lib.components.ArmComponent
import org.ghrobotics.lib.components.EmergencyHandleable
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
//import org.ghrobotics.lib.mathematics.threedim.geometry.Vector3
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.motors.ctre.FalconCTRE.LimitSwitchConfig
import org.mockito.Mockito


/**
 * Construct a joint. The Motors passed in are assumed to already have
 * their remote feedback sensor set up. Furthermore the master is
 * assuemd to be index 0. BE SURE TO CONFIGURE THE REMOTE FEEDBACK SENSOR
 * IN THE MOTORS' DECLARATION! I can't do that here because FalconMotor
 * no do dat. Also, plz configure the encoder to be connected to da master.
 */
class Joint (
        override val motor : FalconMotor<Rotation2d>,
        override val followerMotors: List<FalconMotor<*>>? = null,
//        val encoder : FalconEncoder<T> = motors[0].encoder,
//        override val arbitraryFeedForward : (pos : Double, acceleration: Double) -> Double,
//        val kArmLength: Length,
        override val armKg: Double,
//        override val armKa: Double,
        override val armLength: Double,
        override val armAxleOffset: Translation3d,
        override val armRotationAxis: Translation3d, // for a joint on Croissant, it should be (0,0,1). Make sure it's normalized!
        val kStartPosIsh : Rotation2d,
        val cruiseVel : Velocity<Rotation2d>,
        val cruiseAccel : Acceleration<Rotation2d>,
        val pidfSlot : Int,
        val minLimitConfig: LimitSwitchConfig,
        val maxLimitConfig: LimitSwitchConfig,
        val minPosition : NativeUnit,
        val maxPosition : NativeUnit,
        val model : NativeUnitModel<Rotation2d>,
        val slotConfiguration: SlotConfiguration = SlotConfiguration(),
        val currentLimitConfig: FalconSRX.CurrentLimitConfig = FalconSRX.CurrentLimitConfig(40.amp, 1.second, 35.amp)
) : ArmComponent(Translation3d.kZero, Translation3d.kZero, armLength), MultiMotorTransmission, EmergencyHandleable {

    val encoder = motor.encoder

    init {

        MotorHelpers.configureMotor(
                motor, cruiseVel, cruiseAccel, pidfSlot, currentLimitConfig,
                slotConfiguration, minLimitConfig, maxLimitConfig, minPosition, maxPosition
        )

        var result = true
        followerMotors?.forEach {

            MotorHelpers.configureMotor(
                    it, cruiseVel, cruiseAccel, pidfSlot, currentLimitConfig,
                    slotConfiguration, minLimitConfig, maxLimitConfig, minPosition, maxPosition
            )

            result = result && it.follow(motor)
        }
        assert(result)
    }

    fun resetPosition(newPos : Rotation2d) {
        encoder.resetPosition(model.toNativeUnitPosition(newPos).value)
    }

    var untypedPosition : Double
        get() = encoder.position
        set(newValue) {encoder.resetPosition(newValue)}

    override fun activateEmergency(severity: EmergencyHandleable.Severity) {

        when (severity) {
            is EmergencyHandleable.Severity.DisableOutput -> {
                when(motor) {
                    is FalconCTRE -> {
                        motor.motorController.configPeakOutputReverse(0.0, 0)
                        motor.motorController.configPeakOutputForward(0.0, 0)
                    }
                    is FalconMAX -> {
                        motor.controller.setOutputRange(0.0, 0.0)
                    }
                }

                followerMotors?.forEach {
                    when(it) {
                        is FalconCTRE -> {
                            it.motorController.configPeakOutputReverse(0.0, 0)
                            it.motorController.configPeakOutputForward(0.0, 0)
                        }
                        is FalconMAX -> {
                            it.controller.setOutputRange(0.0, 0.0)
                        }
                    }
                }
            }
            is EmergencyHandleable.Severity.DisableClosedLoop -> {
                when(motor) {
                    is FalconCTRE -> {
                        motor.motorController.configClosedLoopPeakOutput(pidfSlot, 0.0, 0)
                    }
                    is FalconMAX -> {
                        motor.controller.setOutputRange(0.0, 0.0)
                    }
                }

                followerMotors?.forEach {
                    when (it) {
                        is FalconCTRE -> {
                            it.motorController.configClosedLoopPeakOutput(pidfSlot, 0.0, 0)
                        }
                        is FalconMAX -> {
                            it.controller.setOutputRange(0.0, 0.0)
                        }
                    }
                }
            }
        }
    }

    override fun recoverFromEmergency() {
        when(motor) {
            is FalconCTRE -> {
                motor.motorController.configPeakOutputReverse(-1.0, 0)
                motor.motorController.configPeakOutputForward(1.0, 0)
                motor.motorController.configClosedLoopPeakOutput(pidfSlot, 1.0, 0)
            }
            is FalconMAX -> {
                motor.controller.setOutputRange(-1.0, 1.0)
            }
        }

        followerMotors?.forEach {
            when(it) {
                is FalconCTRE -> {
                    it.motorController.configPeakOutputReverse(-1.0, 0)
                    it.motorController.configPeakOutputForward(1.0, 0)
                    it.motorController.configClosedLoopPeakOutput(pidfSlot, 1.0, 0)
                }
                is FalconMAX -> {
                    it.controller.setOutputRange(-1.0, 1.0)
                }
            }
        }
    }

    companion object {

        fun getMockJoint() : Joint {
            return Joint(
                    Mockito.mock(FalconSRX<Rotation2d>(0, NativeUnitRotationModel(4096.nativeUnits)).javaClass),
                    null,
                    0.5,
                    12.inch.meter,
                    Translation3d.kZero,
                    Translation3d(0.0,0.0,1.0),
                    0.degree,
                    300.degree.velocity,
                    3000.degree.acceleration,
                    0,
                    FalconCTRE.LimitSwitchConfig(RemoteLimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled),
                    FalconCTRE.LimitSwitchConfig(RemoteLimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled),
                    -2048.nativeUnits,
                    2048.nativeUnits,
                    NativeUnitRotationModel(4096.nativeUnits),
                    SlotConfiguration()
            )
        }

    }

}