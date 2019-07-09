package org.team5940.pantry.lib

import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.runBlocking
import org.ghrobotics.lib.utils.Source

class FalconChannel<T>(defaultValue: T, capacity: Int = -1) : Source<T> {

    val wrappedValue = Channel<T>(capacity)

    var oldValue = defaultValue

    suspend fun send(newValue: T) =
        wrappedValue.send(newValue)

    suspend fun receive(): T {
        return if (wrappedValue.isEmpty) {
            println("returning oldValue of $oldValue")
            // return the default value
            oldValue
        } else {
            // invoke the channel to get the next value
            val received = wrappedValue.receive()
            oldValue = received
            println("returning new value of $received")
            received
        }
    }

    override fun invoke() = runBlocking { receive() }
}