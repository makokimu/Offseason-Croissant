package org.team5940.pantry.lib

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.lib.wrappers.hid.FalconHIDBuilder
import org.ghrobotics.lib.wrappers.hid.FalconHIDButtonBuilder
import org.ghrobotics.lib.wrappers.hid.HIDButton
import org.ghrobotics.lib.wrappers.hid.HIDSource

fun Number.boundTo(min: Number, max: Number) = when {
    toDouble() > max.toDouble() -> max.toDouble()
    toDouble() < min.toDouble() -> min.toDouble()
    else -> toDouble()
}

/**
 * This button will only trigger if the axis is negative, whereas the stock button()
 * function will trigger if the absolute value exceeds the @param threshold
 */
fun <T : GenericHID> FalconHIDBuilder<T>.lessThanAxisButton(
        hid: T,
        axisId: Int,
        threshold: Double = HIDButton.DEFAULT_THRESHOLD,
        block: FalconHIDButtonBuilder.() -> Unit = {}
) = button(BoundedHIDAxisSource(hid, axisId, 1.0, -1.0, 0.0), threshold, block)

/**
 * This button will only trigger if the axis is positive, whereas the stock button()
 * function will trigger if the absolute value exceeds the @param threshold
 */
fun <T : GenericHID> FalconHIDBuilder<T>.greaterThanAxisButton(
        hid: T,
        axisId: Int,
        threshold: Double = HIDButton.DEFAULT_THRESHOLD,
        block: FalconHIDButtonBuilder.() -> Unit = {}
) = button(BoundedHIDAxisSource(hid, axisId, 1.0, 0.0, 1.0), threshold, block)


class BoundedHIDAxisSource(
        private val genericHID: GenericHID,
        private val axisId: Int,
        private val axisMultiplier: Double = 1.0,
        private val minValue: Double, private val maxValue: Double
) : HIDSource {
    override fun invoke(): Double {

        return (axisMultiplier * genericHID.getRawAxis(axisId)).boundTo(minValue, maxValue)
    }
}