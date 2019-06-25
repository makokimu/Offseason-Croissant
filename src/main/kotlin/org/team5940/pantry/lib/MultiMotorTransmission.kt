package org.team5940.pantry.lib

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX


abstract class MultiMotorTransmission<T: SIUnit<T>>(
        private val master: FalconMotor<T>, vararg followers: FalconMotor<*>)
    : FalconMotor<T> {

    protected open val allMotors: List<FalconMotor<*>> = listOf(master) + followers

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
        allMotors.forEach{
            result = result.and(it.follow(motor))
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
        allMotors.forEach { motor ->
//            motor.kP = 0.0
//            motor.kD = 0.0

            when(motor) {
                is FalconCTRE -> {motor.motorController.run {
                    config_kP(0, 0.0, 0)
                    config_kI(0, 0.0, 0)
                    config_kD(0, 0.0, 0)
                }}
                is FalconMAX -> {motor.controller.p = 0.0; motor.controller.i = 0.0; motor.controller.d = 0.0; }
            }

        }
    }

    abstract fun setClosedLoopGains()

    val outputCurrent = when(master) {
        is FalconCTRE -> {
            when(master.motorController) {
                is IMotorControllerEnhanced -> {
                    (master.motorController as IMotorControllerEnhanced).outputCurrent
                } else -> 0.0
            }
        }
        is FalconMAX -> {
            master.canSparkMax.outputCurrent
        }
        else -> 0.0
    }

    fun FalconMotor<*>.setClosedLoopGains(p: Double, d: Double) {
        when(this) {
            is FalconCTRE -> {this.motorController.run {
                config_kP(0, p, 0)
                config_kI(0, 0.0, 0)
                config_kD(0, d, 0)
            }}
            is FalconMAX -> {this.controller.p = p; this.controller.i = 0.0; this.controller.d = d;
            }
        }
    }
}