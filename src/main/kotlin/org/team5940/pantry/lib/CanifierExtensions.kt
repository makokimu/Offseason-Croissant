package org.team5940.pantry.lib

import com.ctre.phoenix.CANifier
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.utils.Source

fun <K: SIKey> CANifier.asPWMSource(min: Pair<Double, SIUnit<K>>, max: Pair<Double, SIUnit<K>>,
                                    channel: CANifier.PWMChannel): Source<SIUnit<K>> = {
    val currentPWM = doubleArrayOf(0.0, 0.0)
    getPWMInput(channel, currentPWM)
    val dutyCycle = currentPWM[0]
    val interpolatedPosition = min.second.lerp(max.second, dutyCycle - min.first)

    interpolatedPosition
}