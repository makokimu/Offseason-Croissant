@file:Suppress("LeakingThis")

package org.team5940.pantry.lib

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced
import edu.wpi.first.wpilibj.experimental.command.CommandScheduler
import edu.wpi.first.wpilibj.experimental.command.SendableSubsystemBase
import frc.robot.Ports
import frc.robot.subsystems.superstructure.Wrist
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.EmergencyHandleable


/**
 * A MultiMotorTransmission which extends Subsystem. By default,
 * this subsystem will be unregistered
 */
abstract class MultiMotorTransmission<T: SIUnit<T>>(unregisterSubsystem: Boolean = false) : FalconMotor<T>, SendableSubsystemBase(),
        EmergencyHandleable, ConcurrentlyUpdatingComponent {

    abstract val master: FalconMotor<T>
    protected open val followers: List<FalconMotor<*>>? = null

    init {
        if(unregisterSubsystem) CommandScheduler.getInstance().unregisterSubsystem(this)
    }

    override val encoder = master.encoder
    override var motionProfileAcceleration = master.motionProfileAcceleration
    override var motionProfileCruiseVelocity = master.motionProfileCruiseVelocity
    override var outputInverted = master.outputInverted
    override var useMotionProfileForPosition = master.useMotionProfileForPosition
    override var voltageCompSaturation = master.voltageCompSaturation
    override val voltageOutput = master.voltageOutput
    override var brakeMode = master.brakeMode

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
        val motorList: List<FalconMotor<*>> = if(list == null) listOf(master) else list + master

        motorList.forEach { motor ->

            when(motor) {
                is FalconCTRE<*> -> {motor.motorController.run {
                    config_kP(0, 0.0, 0)
                    config_kI(0, 0.0, 0)
                    config_kD(0, 0.0, 0)
                    configClosedLoopPeakOutput(0, 0.0, 0)
                }}
                is FalconMAX<*> -> {motor.controller.p = 0.0; motor.controller.i = 0.0; motor.controller.d = 0.0;
                    motor.controller.setOutputRange(0.0, 0.0)}
            }

        }
    }

    abstract fun setClosedLoopGains()

    val outputCurrent: Double
        get() {
            val master_: FalconMotor<T> = master

            return when(master_) {
                is FalconCTRE -> {
                    when(master_.motorController) {
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
        when(this) {
            is FalconCTRE -> {this.motorController.run {
                config_kP(0, p, 0)
                config_kI(0, 0.0, 0)
                config_kD(0, d, 0)
                config_kF(0, ff, 0)
            }}
            is FalconMAX -> {this.controller.p = p; this.controller.i = 0.0; this.controller.d = d; this.controller.ff = ff}
        }
    }

    override fun activateEmergency() = zeroClosedLoopGains()
    override fun recoverFromEmergency() = setClosedLoopGains()

    data class State(
            val position: Double, // the position in [T] units
            val velocity: Double,
            val acceleration: Double = 0.0
    )

    var currentState = State(0.0, 0.0, 0.0)

    override fun updateState() {
        synchronized(currentState) {
            val lastState = currentState
            val velocity = encoder.velocity
            currentState = State(encoder.position, velocity, velocity - lastState.velocity)
        }
    }
}
