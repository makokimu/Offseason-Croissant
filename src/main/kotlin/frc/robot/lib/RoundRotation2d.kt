package frc.robot.lib

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.kEpsilon

val Number.roundRadian get() = RoundRotation2d(toDouble())
val Number.roundDegree get() = Math.toRadians(toDouble()).roundRadian

class RoundRotation2d : SIUnit<RoundRotation2d> {

    override val value: Double
    val cos: Double
    val sin: Double

    constructor(value: Double) {
        x = Math.cos(value)
        y = Math.sin(value)
        val magnitude = Math.hypot(x, y)
        if (magnitude > kEpsilon) {
            sin = y / magnitude
            cos = x / magnitude
        } else {
            sin = 0.0
            cos = 1.0
        }

        this.value = value
    }

    val radian get() = value // should be between -PI and PI already. // % (Math.PI * 2)
    val degree get() = value / (2 * Math.PI) * 360

    fun isParallel(rotation: RoundRotation2d) = (this - rotation).radian epsilonEquals 0.0

    override fun plus(other: RoundRotation2d): RoundRotation2d {
        return RoundRotation2d(
                this.value + other.value
        )
    }

    override fun minus(other: RoundRotation2d) = plus(-other)

    override fun createNew(newValue: Double) = RoundRotation2d(newValue)

    override fun equals(other: Any?) = other is RoundRotation2d && this.value epsilonEquals other.value

    override fun hashCode() = this.value.hashCode()

    companion object {
        val kZero = RoundRotation2d(0.0)
        val kRotation = 360.roundDegree
    }
}