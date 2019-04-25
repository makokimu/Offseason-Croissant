package frc.robot.subsystems.drive

import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.motors.FalconEncoder
import org.ghrobotics.lib.motors.FalconMotor

/**
 * Create a transmission. The motors should already be inverted and phases set etc before being passed.
 * @param allMotors, with [0] being the master
 */
class LinearTransmission (
        val allMotors : List<FalconMotor<Length>>
    ) : FalconMotor<Length> {

    override val encoder: FalconEncoder<Length>
        get() = master.encoder
    override var motionProfileAcceleration: Double
        get() = master.motionProfileAcceleration
        set(value) {master.motionProfileAcceleration = value}
    override var motionProfileCruiseVelocity: Double
        get() = master.motionProfileCruiseVelocity
        set(value) {master.motionProfileCruiseVelocity = value}
    override var outputInverted: Boolean
        get() = master.outputInverted
        set(value) {master.outputInverted = value}
    override var useMotionProfileForPosition: Boolean
        get() = master.useMotionProfileForPosition
        set(value) {master.useMotionProfileForPosition = value}
    override var voltageCompSaturation: Double
        get() = master.voltageCompSaturation
        set(value) {master.voltageCompSaturation = value}
    override val voltageOutput: Double
        get() = master.voltageOutput

    override fun follow(motor: FalconMotor<*>): Boolean {
       var result = true
        allMotors.forEach{
           result = result.and(it.follow(motor))
        }
        return result
    }

    override fun setDutyCycle(dutyCycle: Double, arbitraryFeedForward: Double) {
        master.setDutyCycle(dutyCycle, arbitraryFeedForward)
    }

    override fun setNeutral() {
        master.setNeutral()
    }

    override fun setPosition(position: Double, arbitraryFeedForward: Double) {
        master.setPosition(position, arbitraryFeedForward)
    }

    override fun setVelocity(velocity: Double, arbitraryFeedForward: Double) {
        master.setVelocity(velocity, arbitraryFeedForward)
    }

    override fun setVoltage(voltage: Double, arbitraryFeedForward: Double) {
        master.setVoltage(voltage, arbitraryFeedForward)
    }

    private val master = allMotors[0]

    fun stop() {
        allMotors.forEach {
            it.setDutyCycle(0.0)
//            motor.percentOutput = 0.0
        }
    }

    override var brakeMode: Boolean
        get() = master.brakeMode
        set(value) {master.brakeMode = value}


//    override fun setVelocityAndArbitraryFeedForward(velocity: Velocity<Length>, arbitraryFeedForward: Double) {
//        master.setVelocityAndArbitraryFeedForward(velocity, arbitraryFeedForward)
//    }

//    override fun setVelocityAndArbitraryFeedForward(velocity: Double, arbitraryFeedForward: Double) {
//        master.setVelocityAndArbitraryFeedForward(velocity, arbitraryFeedForward)
//    }

}