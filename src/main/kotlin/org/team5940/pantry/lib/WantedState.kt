package org.team5940.pantry.lib

sealed class WantedState {
    object Nothing : WantedState()

    class Position(val targetPosition: Double) : WantedState() {
        operator fun plus(delta: Double) = Position(targetPosition + delta)

        fun coerceIn(range: ClosedFloatingPointRange<Double>) = Position(
                targetPosition.coerceIn(range)
        )
    }

    class Velocity(val targetVelocity: Double) : WantedState()

    abstract class CustomState : WantedState() {
        abstract fun useState(wantedState: WantedState)
    }
}