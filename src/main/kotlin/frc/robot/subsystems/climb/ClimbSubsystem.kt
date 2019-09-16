package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import frc.robot.subsystems.superstructure.Length
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import kotlin.math.abs

object ClimbSubsystem: FalconSubsystem() {

    val stiltMotor = FalconMAX(99999, CANSparkMaxLowLevel.MotorType.kBrushless,
           // the encoder is attached behind a 1:9 versaplanetary and a 1:2 pulley thing
            NativeUnitLengthModel(
                    1.nativeUnits * 9.0 * 2.0,
                    1.5.inch / 2
            )
            ).apply {

        setPIDGains(0.0, 0.0)
        encoder.resetPosition(0.meter)
        canSparkMax.setSmartCurrentLimit(30, 30) // TODO check
        canSparkMax.burnFlash()
    }

    val intakeWheels = FalconSRX(99999, DefaultNativeUnitModel)

    private val wantedStateMutex = Object()
    private val currentStateMutex = Object()
    val currentState = MultiMotorTransmission.State(0.meter)
        get() = synchronized(currentStateMutex) { field }

    var wantedState: WantedState = WantedState.Nothing
        get() = synchronized(wantedStateMutex) { field }
        set(newValue) = synchronized(wantedStateMutex) { field = newValue }

    /**
     * Determine if the joint is within the [tolerance] of the current wantedState.
     * If the wantedState isn't [WantedState.Position<*>], return false.
     */
    fun isWithTolerance(tolerance: SIUnit<Meter>): Boolean {
        val state = wantedState as? WantedState.Position<*> ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition.value - currentState.position.value) < tolerance.value
    }

}

fun <K: SIKey> FalconMAX<K>.setPIDGains(p: Double, d: Double, ff: Double = 0.0) {
    controller.p = p
    controller.d = d
    controller.ff = ff
    controller.i = 0.0
}