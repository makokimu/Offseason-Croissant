package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.frc2.command.RunCommand
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Controls
import frc.robot.auto.routines.withExit
import frc.robot.subsystems.superstructure.*
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.hid.getY
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
        encoder.resetPosition(25.inch)
        canSparkMax.setSmartCurrentLimit(30, 30) // TODO check
        brakeMode = false
//        canSparkMax.burnFlash()
    }

    val prepMove = sequential {
        +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
        +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 93.degree) // TODO check preset
        val move = SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 93.degree) // TODO check preset
        +parallel {
            +move
            +(RunCommand(Runnable{ intakeWheels.setDutyCycle(0.3)}, ClimbSubsystem).withExit { !move.isScheduled }.whenFinished { intakeWheels.setNeutral()})
        }
    }

    val fullS3ndClimbCommand = object : FalconCommand(ClimbSubsystem,
            Elevator, Proximal, Wrist, Superstructure) {

        val targetHeight = 16.inch

        override fun initialize() {
            stiltMotor.controller.setOutputRange(-0.55, 0.55)
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 0.3)
            Proximal.wantedState = WantedState.Position((-15).degree)
            Elevator.setPositionMode()
            Proximal.setPositionMode()
            Wrist.wantedState = WantedState.Position(85.degree)
        }
        override fun execute() {
            stiltMotor.setPosition(targetHeight)
            Elevator.wantedState = WantedState.Position(targetHeight)
            intakeWheels.setDutyCycle(0.7)

            println("Elevator pos ${Elevator.motor.encoder.position.inch} Prox pos ${Proximal.motor.encoder.position.degree} " +
                    "Prox output ${Proximal.motor.master.talonSRX.motorOutputPercent} Hab climber pos ${stiltMotor.encoder.position.inch}")
        }
        override fun isFinished() = stiltMotor.encoder.position < targetHeight + 0.5.inch
                && Elevator.motor.encoder.position < targetHeight + 0.5.inch
        override fun end(interrupted: Boolean) {
            Elevator.setMotionMagicMode()
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
            Proximal.setMotionMagicMode()
            Elevator.setNeutral()
            stiltMotor.setNeutral()
            intakeWheels.setNeutral()
        }
    }

    override fun lateInit() {
        SmartDashboard.putData("test move", sequential {
            +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 80.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 95.degree) // TODO check preset
        })

        SmartDashboard.putData("closed loop climb", fullS3ndClimbCommand)

        SmartDashboard.putData("straight out", Superstructure.kHatchMid)
        SmartDashboard.putData("SketchyTest", SketchyTest())
    }

    val safeRange = (10.inch..33.inch)

    val intakeWheels = FalconSRX(45, DefaultNativeUnitModel).apply {
        outputInverted = true
        configCurrentLimit(true, FalconSRX.CurrentLimitConfig(30.amp, 2.second, 10.amp))
    }

    private val wantedStateMutex = Object()
    private val currentStateMutex = Object()
    val currentState = MultiMotorTransmission.State(0.meter)
        get() = synchronized(currentStateMutex) { field }

//    var wantedState: WantedState = WantedState.Nothing
//        get() = synchronized(wantedStateMutex) { field }
//        set(newValue) = synchronized(wantedStateMutex) { field = newValue }

//    /**
//     * Determine if the joint is within the [tolerance] of the current wantedState.
//     * If the wantedState isn't [WantedState.Position<*>], return false.
//     */
//    fun isWithTolerance(tolerance: SIUnit<Meter>): Boolean {
//        val state = wantedState as? WantedState.Position<*> ?: return false // smart cast state, return false if it's not Position
//
//        return abs(state.targetPosition.value - currentState.position.value) < tolerance.value
//    }

}

fun <K: SIKey> FalconMAX<K>.setPIDGains(p: Double, d: Double, ff: Double = 0.0) {
    controller.p = p
    controller.d = d
    controller.ff = ff
    controller.i = 0.0
}