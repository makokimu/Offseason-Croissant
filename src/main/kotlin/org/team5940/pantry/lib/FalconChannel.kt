package org.team5940.pantry.lib

import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map

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

fun <E> Channel<E>.receiveNonBlocking(): E? {
    return if (this.isEmpty) null else runBlocking { receive() }
}

suspend fun <E> Channel<E>.clearAndS3NDIT(it: E) {
    while(!this.isEmpty) { receive() }
    if(isEmpty) send(it) else { receive() ; send(it) }
}

fun <E> Channel<E>.recieveOrLastValue(lastValue: E): E {
    return if (this.isEmpty) lastValue else runBlocking { receive() }
}

fun <E> S3nd(element: E): E {
    return element
}

suspend fun <E> Channel<E>.clear() {
    while(!this.isEmpty) {
        receive()
    }
}

/**
 * Launch a coroutine which will clear the channel and s3nd the [value], hopefully
 */
fun <E> Channel<E>.launchAndSend(value: E) = FishyRobot.updateScope.launch {
    clear() // clear the channel, hopefully
    send(value) // S3NDIT
}

infix fun <E> E.s3ndIntoBlocking(channel: Channel<E>) = runBlocking { channel.send(this@s3ndIntoBlocking) }