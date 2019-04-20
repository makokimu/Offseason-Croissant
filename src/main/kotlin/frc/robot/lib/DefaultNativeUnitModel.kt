package frc.robot.lib

import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel

object DefaultNativeUnitModel : NativeUnitModel<NativeUnit>(NativeUnit.kZero) {
    override fun fromNativeUnitPosition(nativeUnits: Double): Double = nativeUnits
    override fun toNativeUnitPosition(modelledUnit: Double): Double = modelledUnit
}