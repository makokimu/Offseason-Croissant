package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration
import com.revrobotics.CANDigitalInput
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTREEncoder
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.motors.rev.FalconMAXEncoder
import org.team5940.pantry.exparimental.command.SendableSubsystemBase
import org.ghrobotics.lib.motors.ctre.FalconCTRE.LimitSwitchConfig


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
        val arbitraryFeedForward : (pos : Double, acceleration: Double) -> Double,
        val kStartPosIsh : T,
        val cruiseVel : Velocity<T>,
        val cruiseAccel : Acceleration<T>,
        val pidfSlot : Int,
        val minLimitConfig: LimitSwitchConfig,
        val maxLimitConfig: LimitSwitchConfig,
        val minPosition : NativeUnit,
        val maxPosition : NativeUnit,
        val slotConfiguration: SlotConfiguration = SlotConfiguration(),
        val currentLimitConfig: FalconSRX.CurrentLimitConfig = FalconSRX.CurrentLimitConfig(40.amp, 1.second, 35.amp)
) : SendableSubsystemBase() {

    val master = motors[0]

    val encoder = master.encoder

    var lastVelocity = 0.0

    override fun periodic() {
        val mVelocity = untypedVelcoity
        val mAccel = mVelocity - lastVelocity

        master.setPosition(position.value, arbitraryFeedForward.invoke(untypedPosition, mAccel))

        lastVelocity = mVelocity
    }

    init {

        motors.forEach{

            it.motionProfileCruiseVelocity = cruiseVel.value
            it.motionProfileCruiseVelocity = cruiseAccel.value
            it.useMotionProfileForPosition = true
            it.brakeMode = true

            when (it) {
                is FalconSRX -> {
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
                    motor.configClosedLoopPeriod(pidfSlot, slotConfiguration.closedLoopPeriod, 0)

                    motor.configForwardLimitSwitchSource(maxLimitConfig.source, maxLimitConfig.limitNormal, motor.deviceID, 0)
                    motor.configReverseLimitSwitchSource(minLimitConfig.source, minLimitConfig.limitNormal, motor.deviceID, 0)

                    motor.configForwardSoftLimitThreshold(it.model.fromNativeUnitPosition(maxPosition.value).toInt(), 0)
                    motor.configReverseSoftLimitThreshold(it.model.fromNativeUnitPosition(minPosition.value).toInt(), 0)

                    motor.configForwardSoftLimitEnable(true, 0)
                    motor.configReverseSoftLimitEnable(true, 0)

                }
                is FalconMAX -> {
                    val motor = it.canSparkMax
                    val pid = it.controller

                    pid.setOutputRange(-1.0, 1.0)
                    pid.setP(slotConfiguration.kP, pidfSlot)
                    pid.setI(slotConfiguration.kI, pidfSlot)
                    pid.setD(slotConfiguration.kD, pidfSlot)
                    pid.setFF(slotConfiguration.kF, pidfSlot)
                    pid.setIZone(slotConfiguration.integralZone.toDouble(), pidfSlot)
                    pid.setIMaxAccum(slotConfiguration.maxIntegralAccumulator, pidfSlot)
                    pid.setSmartMotionAllowedClosedLoopError(slotConfiguration.allowableClosedloopError.toDouble(), pidfSlot)

                    val maxPolarity = if (maxLimitConfig.limitNormal == LimitSwitchNormal.NormallyOpen) CANDigitalInput.LimitSwitchPolarity.kNormallyClosed else CANDigitalInput.LimitSwitchPolarity.kNormallyOpen
                    val minPolarity = if (minLimitConfig.limitNormal == LimitSwitchNormal.NormallyOpen) CANDigitalInput.LimitSwitchPolarity.kNormallyClosed else CANDigitalInput.LimitSwitchPolarity.kNormallyOpen

                    motor.getForwardLimitSwitch(maxPolarity)
                    motor.getReverseLimitSwitch(minPolarity)

                }
                else -> TODO("Config not implemented for non-CTR and non-REV joint!")
            }



        }

//        setpoint = position



    }

    /**
     * getter/setter for the position as a typed unit of type [T]
     * Getting this will get the position [T] of the joint
     * Setting this will set the target motion profile position of the joint
     */
    var position : T = kStartPosIsh
        get() = when (encoder) {
            is FalconCTREEncoder -> encoder.model.fromNativeUnitPosition(encoder.rawPosition.nativeUnits)
            is FalconMAXEncoder -> encoder.model.fromNativeUnitPosition(encoder.rawPosition.nativeUnits)
//            else -> kZero
            else -> TODO("Implement setting with other motor controllers?")
        }

    val velocity : Velocity<T>
        get() = kStartPosIsh.createNew(encoder.velocity).velocity

    val untypedVelcoity : Double
        get() = encoder.velocity

    fun resetPosition(newPos : T) {
        when (encoder) {
            is FalconCTREEncoder -> encoder.resetPosition(encoder.model.toNativeUnitPosition(newPos).value)
            is FalconMAXEncoder -> encoder.resetPosition(encoder.model.toNativeUnitPosition(newPos).value)
            else -> TODO("Encoders for non-CTR and non-REV encoders not implemented")
        }
    }

    var untypedPosition : Double
        get() = encoder.position
        set(newValue) {encoder.resetPosition(newValue)}
}