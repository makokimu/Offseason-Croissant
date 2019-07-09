package org.team5940.pantry.lib

sealed class WantedState {
    object Nothing : WantedState()

    class Position(val targetPosition: Double) : WantedState()

    class Velocity(val targetVelocity: Double) : WantedState()

    abstract class CustomState : WantedState() {
        abstract fun useState()
    }
}