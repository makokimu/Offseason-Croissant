package org.team5940.pantry.lib

interface ConcurrentlyUpdatingComponent {

    suspend fun updateState()

    fun useState() {}
}
