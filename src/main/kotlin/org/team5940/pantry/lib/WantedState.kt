package org.team5940.pantry.lib

sealed class WantedState {
    object Nothing : WantedState() {
        override fun toString(): String = "Nothing"
    }

    class Position(val targetPosition: Double) : WantedState() {
        operator fun plus(delta: Double) = Position(targetPosition + delta)

        fun coerceIn(range: ClosedFloatingPointRange<Double>) = Position(
                targetPosition.coerceIn(range)
        )
        override fun toString(): String = "Position $targetPosition"
    }

    class Velocity(val targetVelocity: Double) : WantedState() {
        override fun toString(): String = "Velocity $targetVelocity"
    }

    class Voltage(val output: Double) : WantedState() {
        override fun toString() = "Voltage $output"
    }

    abstract class CustomState : WantedState() {
        abstract fun useState(wantedState: WantedState)
        override fun toString(): String = "CustomState"
    }
}