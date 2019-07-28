@file:Suppress("EXPERIMENTAL_API_USAGE")

package org.team5940.pantry.lib

import kotlinx.coroutines.channels.Channel
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

    internal val wantedStateChannel = Channel<WantedState>(Channel.CONFLATED)

    open val currentState: MultiMotorTransmission.State
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
    open var wantedState: WantedState = WantedState.Nothing
//        set(value) {
//            wantedStateChannel.launchAndSend(value)
//            field = value
//        }
        @Synchronized get
        @Synchronized set

    /**
     * [lastWantedState] is accessed by coroutines and as such shouldn't be touched by the main thread
     */
    internal var lastWantedState = wantedState

    /**
     * Calculate the arbitrary feed forward given the [currentState] in Volts
     */
    open fun calculateFeedForward(currentState: JointState) = 0.0

    open fun customizeWantedState(wantedState: WantedState) = wantedState

    override suspend fun updateState() = motor.updateState()
    override fun useState() {
//        val newState = if(wantedStateChannel.isEmpty) lastWantedState else wantedStateChannel.receive()
//        if(lastWantedState != newState) lastWantedState = newState

//        val channel = wantedStateChannel
//        val newState = channel.poll() ?: lastWantedState
//        if(lastWantedState != newState) lastWantedState = newState

        val newState = wantedState

//        println("new wanted state is $newState")

        val customizedState = customizeWantedState(newState)
        val feedForward = calculateFeedForward(currentState)

        motor.s3ndState(customizedState, feedForward)
    }
}

interface ConcurrentlyUpdatingJoint {

    suspend fun updateState(): JointState

    fun useState() {}
}
