package org.team5940.pantry.lib

import kotlinx.coroutines.channels.Channel
import org.ghrobotics.lib.utils.Source

class FalconConflatedChannel<T>(defaultValue: T) : Source<T> {

    private val wrappedValue = Channel<T>(Channel.CONFLATED)

    var lastValue = defaultValue

    fun offer(newValue: T) = wrappedValue.offer(newValue)

    fun poll(): T {
        val queuedMessage = wrappedValue.poll() ?: lastValue
        if (lastValue != queuedMessage) lastValue = queuedMessage
        return queuedMessage
    }

    override fun invoke() = poll()
}
