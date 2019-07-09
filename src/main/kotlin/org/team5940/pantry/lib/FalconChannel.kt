package org.team5940.pantry.lib

import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.runBlocking
import org.ghrobotics.lib.utils.Source

@Deprecated("Channels bad fite me")
class FalconChannel<T>(defaultValue: T, capacity: Int = -1) : Source<T> {

    val wrappedValue = Channel<T>(capacity)

    var oldValue = defaultValue

    suspend fun send(newValue: T) =
        wrappedValue.send(newValue)

    suspend fun receive(): T {
        return if (wrappedValue.isEmpty) {
            // return the default value
            oldValue
        } else {
            // invoke the channel to get the next value
            val received = wrappedValue.receive()
            oldValue = received
            received
        }
    }

    override fun invoke() = runBlocking { receive() }
}