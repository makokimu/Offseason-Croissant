package org.team5940.pantry.lib

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.EmergencyHandleable

/**
 * A MultiMotorTransmission contains multiple motors.
 * It is paramatrized with a SIUnit of type [T] (ex. Length),
 * and a master of type [M] with a model of type [T]
 */
abstract class MultiMotorTransmission<T : SIUnit<T>, M : FalconMotor<T>> : FalconMotor<T>,
        EmergencyHandleable, ConcurrentlyUpdatingJoint {

    abstract val master: M

    protected open val followers: List<FalconMotor<*>>? = null

    override val encoder by lazy { master.encoder }
//        @Synchronized get
    override var motionProfileAcceleration
        get() = master.motionProfileAcceleration
        set(value) { master.motionProfileAcceleration = value }

    override var motionProfileCruiseVelocity
        get() = master.motionProfileAcceleration
        set(value) { master.motionProfileAcceleration = value }
    override var outputInverted
        get() = master.outputInverted
        set(value) {
            master.outputInverted = value
            followers?.forEach {
                it.outputInverted = value
            }
        }
    override var useMotionProfileForPosition
        get() = master.useMotionProfileForPosition
        set(value) { master.useMotionProfileForPosition = value }
    override var voltageCompSaturation
        get() = master.voltageCompSaturation
        set(value) { master.voltageCompSaturation = value }
    override val voltageOutput
        get() = master.voltageOutput
    override var brakeMode
        get() = master.brakeMode
        set(value) { master.brakeMode = value }

    override fun follow(motor: FalconMotor<*>): Boolean {
        var result = true

        result = result and master.follow(motor)

        val followers_ = followers

        followers_?.forEach {
            result = result and it.follow(motor)
        }

        return result
    }

    override fun setDutyCycle(dutyCycle: Double, arbitraryFeedForward: Double) =
            master.setDutyCycle(dutyCycle, arbitraryFeedForward)

    override fun setNeutral() {
        master.setNeutral()
    }

    override fun setPosition(position: Double, arbitraryFeedForward: Double) =
        master.setPosition(position, arbitraryFeedForward)

    override fun setVelocity(velocity: Double, arbitraryFeedForward: Double) =
        master.setVelocity(velocity, arbitraryFeedForward)

    override fun setVoltage(voltage: Double, arbitraryFeedForward: Double) =
        master.setVoltage(voltage, arbitraryFeedForward)

    fun zeroClosedLoopGains() {
        val list = followers
        val motorList: List<FalconMotor<*>> = if (list == null) listOf(master) else list + master

        motorList.forEach { motor ->

            when (motor) {
                is FalconCTRE<*> -> { motor.motorController.run {
                    config_kP(0, 0.0, 0)
                    config_kI(0, 0.0, 0)
                    config_kD(0, 0.0, 0)
                    configClosedLoopPeakOutput(0, 0.0, 0)
                } }
                is FalconMAX<*> -> { motor.controller.p = 0.0; motor.controller.i = 0.0; motor.controller.d = 0.0
                    motor.controller.setOutputRange(0.0, 0.0) }
            }
        }
    }

    abstract fun setClosedLoopGains()

    val outputCurrent: Double
        get() {
            val master_: FalconMotor<T> = master

            return when (master_) {
                is FalconCTRE -> {
                    when (master_.motorController) {
                        is IMotorControllerEnhanced -> {
                            (master_.motorController as IMotorControllerEnhanced).outputCurrent
                        } else -> 0.0
                    }
                }
                is FalconMAX -> {
                    master_.canSparkMax.outputCurrent
                }
                else -> 0.0
            }
        }

    fun FalconMotor<*>.setClosedLoopGains(p: Double, d: Double, ff: Double = 0.0) {
        when (this) {
            is FalconCTRE -> { this.motorController.run {
                config_kP(0, p, 0)
                config_kI(0, 0.0, 0)
                config_kD(0, d, 0)
                config_kF(0, ff, 0)
            } }
            is FalconMAX -> { this.controller.p = p; this.controller.i = 0.0; this.controller.d = d; this.controller.ff = ff }
        }
    }

    override fun activateEmergency() = zeroClosedLoopGains()
    override fun recoverFromEmergency() = setClosedLoopGains()

    data class State(
        val position: Double, // the position in [T] units
        val velocity: Double = 0.0,
        val acceleration: Double = 0.0
    ) {
        companion object {
            val kZero = State(0.0, 0.0)
        }
    }

    internal val currentStateMutex = Object()
    var currentState = State.kZero
        get() = synchronized(currentStateMutex) { field }
        set(newValue) = synchronized(currentStateMutex) { field = newValue }

    private var lastUpdateTime = Timer.getFPGATimestamp()

    override fun updateState(): JointState {
        val now = Timer.getFPGATimestamp()
        val position = encoder.position
        val lastState = currentState
        val velocity = encoder.velocity

        // add the observation to the current state channel
        val newState = State(position, velocity, (velocity - lastState.velocity) / (now - lastUpdateTime))

        lastUpdateTime = now

        return newState
    }

    /**
     * S3nd the wanted demand to the motor RIGHT NOW
     * @param wantedState the wanted state
     * @param arbitraryFeedForward the arbitraryFeedForward in Volts
     */
    fun s3ndState(wantedState: WantedState?, arbitraryFeedForward: Double) {
        if (wantedState == null) return

        when (wantedState) {
            is WantedState.Nothing -> setNeutral()
            is WantedState.Position -> setPosition(wantedState.targetPosition, arbitraryFeedForward)
            is WantedState.Velocity -> setVelocity(wantedState.targetVelocity, arbitraryFeedForward)
            is WantedState.Voltage -> setVoltage(wantedState.output, arbitraryFeedForward)
            is WantedState.CustomState -> wantedState.useState(wantedState)
        }
    }
}
