package frc.robot.subsystems.drive

import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.wrappers.FalconMotor

/**
 * Create a transmission. The motors should already be inverted and phases set etc before being passed.
 * @param allMotors, with [0] being the master
 */
class Transmission (
        private val allMotors : List<FalconMotor<Length>>
    ) : FalconMotor<Length> {

    private val master = allMotors[0]

    fun stop() {
        allMotors.forEach { motor ->
            motor.percentOutput = 0.0
        }
    }

    // in case we get stuff happening
    fun reconfigFollow() {
        // TODO stuff
    }

    /**
     * Setting this value will command the motor to run at the specified output percentage
     * Getting this value will return the current output percentage of the motor
     */
    override var percentOutput: Double
        get() = master.percentOutput
        set(value) {
            master.percentOutput = value
        }

    /**
     * Setting this value will command the motor to run at the specified velocity
     * Getting this value will return the current velocity of the motor
     */
    override var velocity: Velocity<Length>
        get() = master.velocity
        set(value) {
            master.velocity = value
        }

    override val voltageOutput: Volt
        get() = master.voltageOutput

    /**
     * Setting this value will command the motor to run at the specified velocity
     * Getting this value will return the current velocity of the motor
     */
    override fun setVelocityAndArbitraryFeedForward(velocity: Velocity<Length>, arbitraryFeedForward: Double) {
        master.setVelocityAndArbitraryFeedForward(velocity, arbitraryFeedForward)
    }

}