package org.team5940.pantry.lib

import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.runBlocking
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.subsystems.EmergencyHandleable

typealias JointState = MultiMotorTransmission.State

/**
 * A joint which concurrently updates and sends demands using Channels for
 * both it's current state, a [JointState], and recieves
 * demands of type [WantedState].
 */
abstract class ConcurrentFalconJoint<T : SIUnit<T>, V : FalconMotor<T>> : ConcurrentlyUpdatingJoint,
        LoggableFalconSubsystem(), EmergencyHandleable {

    abstract val motor: MultiMotorTransmission<T, V>

    override fun activateEmergency() = motor.activateEmergency()
    override fun recoverFromEmergency() = motor.recoverFromEmergency()
    override fun setNeutral() {
        wantedState = WantedState.Nothing
        motor.setNeutral() }

    private val wantedStateChannel = Channel<WantedState>(Channel.CONFLATED)

    val currentState: MultiMotorTransmission.State
//        @Log
        get() {
            return motor.currentState
        }

    /**
     * The current wantedState of the joint.
     * Setting this will both set the backing field and s3nd the new demand into the [wantedStateChannel]
     * Getting this will return the field
     *
     * Only get this from the main thread!
     */
//    @Log
    var wantedState: WantedState = WantedState.Nothing
        set(value) {
            runBlocking { wantedStateChannel.send(value) }
            field = value
        }

    /**
     * [lastWantedState] is accessed by coroutines and as such shouldn't be touched by the main thread
     */
    private var lastWantedState = wantedState

    /**
     * Calculate the arbitrary feed forward given the [currentState] in Volts
     */
    open fun calculateFeedForward(currentState: JointState) = 0.0

    open fun customizeWantedState(wantedState: WantedState) = wantedState

    override suspend fun updateState() = motor.updateState()
    override suspend fun useState() {
        val newState = wantedStateChannel.recieveOrLastValue(lastWantedState)
        lastWantedState = newState

        val customizedState = customizeWantedState(newState)
        val feedForward = calculateFeedForward(currentState)

        motor.s3ndState(customizedState, feedForward)
    }
}

interface ConcurrentlyUpdatingJoint {

    suspend fun updateState(): JointState

    suspend fun useState() {}
}
