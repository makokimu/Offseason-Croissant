package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.mathematics.units.derived.degree
import org.junit.Test
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports

class ProximalTranslationTest {

    @Test
    fun testProxTranslation() {

        val state = SuperstructureState(16.5.inch, 0.degree, 4.degree)

        val state2 = SuperstructureState(29.inch, (-70).degree, 36.degree)
//        println(SuperstructurePlanner.worstCaseProximalTipElevation(
//                state, state2
//        ).inch)

        val inches = 15.inch
        val nativeunit = Elevator.motor.master.model.toNativeUnitPosition(inches)
        println(nativeunit.value)
    }
}