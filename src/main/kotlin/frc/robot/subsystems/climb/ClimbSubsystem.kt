package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.superstructure.Length
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.SuperstructurePlanner
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import kotlin.math.abs

object ClimbSubsystem: FalconSubsystem() {

    val stiltMotor: FalconMAX<Meter> = FalconMAX(9, CANSparkMaxLowLevel.MotorType.kBrushless,
           // the encoder is attached behind a 1:9 versaplanetary and a 1:2 pulley thing
            NativeUnitLengthModel(
                    1.nativeUnits * 9.0 * 2.0,
                    1.5.inch / 2
            )
            ).apply {

        setPIDGains(1.0, 0.0)
        encoder.canEncoder.positionConversionFactor = -1.0
        encoder.resetPosition(33.inch)
        canSparkMax.setSmartCurrentLimit(30, 30) // TODO check
//        canSparkMax.burnFlash()
    }

    override fun lateInit() {
        SmartDashboard.putData("test move", sequential {
            +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 80.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 95.degree) // TODO check preset
        })

        SmartDashboard.putData("straight out", Superstructure.kHatchMid)
    }

    val safeRange = (10.inch..33.inch)

    val intakeWheels = FalconSRX(45, DefaultNativeUnitModel).apply {
        outputInverted = true
        configCurrentLimit(true, FalconSRX.CurrentLimitConfig(30.amp, 2.second, 15.amp))
    }

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