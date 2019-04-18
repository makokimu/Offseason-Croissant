package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration
import frc.robot.lib.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.motors.FalconEncoder
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.ctre.FalconCTREEncoder
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAXEncoder
import org.team5940.pantry.exparimental.command.SendableSubsystemBase
import kotlin.math.roundToInt


/**
 * Construct a joint. The Motors passed in are assumed to already have
 * their remote feedback sensor set up. Furthermore the master is
 * assuemd to be index 0. BE SURE TO CONFIGURE THE REMOTE FEEDBACK SENSOR
 * IN THE MOTORS' DECLARATION! I can't do that here because FalconMotor
 * no do dat. Also, plz configure the encoder to be connected to da master.
 */
class Joint<T : SIUnit<T>> (
        val motors : List<FalconMotor<T>>,
//        val encoder : FalconEncoder<T> = motors[0].encoder,
        val arbitraryFeedForward : (pos : T) -> Volt,
        val kZero : T,
        val cruiseVel : Velocity<T>,
        val cruiseAccel : Acceleration<T>,
        val pidfSlot : Int,
        val minPosition : NativeUnit,
        val maxPosition : NativeUnit,
        val slotConfiguration: SlotConfiguration = SlotConfiguration(),
        val currentLimitConfig: FalconSRX.CurrentLimitConfig = FalconSRX.CurrentLimitConfig(40.amp, 1.second, 35.amp)
) : SendableSubsystemBase() {

    val master = motors[0]

    val encoder = master.encoder

    var controlMode : ControlMode = ControlMode.MotionMagic

    var setpoint: T = position

    override fun periodic() {
        master.set(
                controlMode, setpoint,
                DemandType.ArbitraryFeedForward, arbitraryFeedForward.invoke(master.encoder.position.T).value / 12
        )
    }

    init {

        motors.forEach{

            it.motionProfileCruiseVelocity = cruiseVel.value
            it.motionProfileCruiseVelocity = cruiseAccel.value

            when {
                it is FalconSRX -> {
                    it.configCurrentLimit(
                            true,
                            currentLimitConfig
                    )

                    val motor = it.motorController

                    motor.configPeakOutputForward(1.0, 0)
                    motor.configPeakOutputReverse(-1.0, 0)
                    motor.config_kP(pidfSlot, slotConfiguration.kP, 0)
                    motor.config_kI(pidfSlot, slotConfiguration.kI, 0)
                    motor.config_kD(pidfSlot, slotConfiguration.kD, 0)
                    motor.config_kF(pidfSlot, slotConfiguration.kF, 0)
                    motor.config_kF(pidfSlot, slotConfiguration.kF, 0)
                    motor.config_IntegralZone(pidfSlot, slotConfiguration.integralZone, 0)
                    motor.configMaxIntegralAccumulator(pidfSlot, slotConfiguration.maxIntegralAccumulator, 0)
                    motor.configAllowableClosedloopError(pidfSlot, slotConfiguration.allowableClosedloopError, 0)
                    motor.configClosedLoopPeakOutput(pidfSlot, slotConfiguration.closedLoopPeakOutput, 0)
                    motor.


                }
                else -> TODO("Config not implemented for non-CTRE and non-REV joint!")


            }



        }

        setpoint = position

        master.softLimitForward = minPosition
        master.softLimitReverse = maxPosition
        master.softLimitForwardEnabled = true
        master.softLimitReverseEnabled = true


    }

    /**
     * getter/setter for the position as a typed unit of type [T]
     * Getting this will get the position [T] of the joint
     * Setting this will set the sensor position of the joint
     */
    var position : T
        get() = when {
            encoder is FalconCTREEncoder -> encoder.model.fromNativeUnitPosition(encoder.rawPosition.nativeUnits)
            encoder is FalconMAXEncoder -> encoder.model.fromNativeUnitPosition(encoder.rawPosition.nativeUnits)
            else -> kZero
//            else -> null
        }
        set(newValue) {
            when {
                encoder is FalconCTREEncoder -> encoder.resetPosition(encoder.model.toNativeUnitPosition(newValue).value)
                encoder is FalconMAXEncoder -> encoder.resetPosition(encoder.model.toNativeUnitPosition(newValue).value)
                else -> TODO("Implement setting with other motor controllers?")
            }
        }

    var untypedPosition : Double
        get() = encoder.position
        set(newValue) {encoder.resetPosition(newValue)}


}