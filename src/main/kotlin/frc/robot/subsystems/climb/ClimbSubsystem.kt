@file:Suppress("RemoveRedundantQualifierName")

package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import com.team254.lib.physics.DCMotorTransmission
import edu.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.frc2.command.RunCommand
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Controls
import frc.robot.auto.routines.withExit
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.*
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import java.awt.Color

object ClimbSubsystem: FalconSubsystem() {

    val stiltTransmission = DCMotorTransmission(
            594.0 / 12.0 / 18.0, // 18:1 gearing
            2.6 / 12.0 * 18.0, // 18:1 gearing
            0.5 // totally a guess
    )
    val gravityVoltage = 1.3.volt

    val stiltMotor: FalconMAX<Meter> = FalconMAX(9, CANSparkMaxLowLevel.MotorType.kBrushless,
            // the encoder is attached behind a 1:9 versaplanetary and a 1:2 pulley thing
            NativeUnitLengthModel(
                    1.nativeUnits * 7.0 * 3.0 * 2.0,
                    1.5.inch / 2
            )
    ).apply {

        setPIDGains(1.0, 0.0)
        encoder.canEncoder.positionConversionFactor = -1.0
        encoder.resetPosition(kZero)
        canSparkMax.setSmartCurrentLimit(40, 40) // TODO check
        brakeMode = false
//        canSparkMax.burnFlash()
    }
    fun configureVelocityPIDMode() {
        stiltMotor.setPIDGains(0.007, 0.0)
    }

    fun setClimbProfile(state: TrapezoidProfile.State) {
        val motorSpeedRadPerSec = (state.velocity / (1.5.inch.meter * Math.PI) * 2 * Math.PI) // meters per sec divided by meter per circum is revolutions per sec, times 2pi is rad per sec
        val transFeedforward = stiltTransmission.getVoltageForTorque(motorSpeedRadPerSec, 5.0) // idk if 5 is right
        stiltMotor.setPosition(state.position.meter, transFeedforward.volt)
    }

    private val kZero = 25.inch
    fun zero() = stiltMotor.encoder.resetPosition(kZero)

    val prepMove = sequential {
        +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
        +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 93.degree) // TODO check preset
        val move = SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 93.degree) // TODO check preset
        +parallel {
            +move
            +(RunCommand(Runnable{ intakeWheels.setDutyCycle(0.3)}, ClimbSubsystem).withExit { !move.isScheduled }.whenFinished { intakeWheels.setNeutral()})
        }
    }

    override fun periodic() {
        SmartDashboard.putNumber("Stilt pos", ClimbSubsystem.stiltMotor.encoder.position.inch)
        SmartDashboard.putNumber("Stilt amps", ClimbSubsystem.stiltMotor.drawnCurrent.amp)
    }

    val fullS3ndClimbCommand = object : FalconCommand(ClimbSubsystem,
            Elevator, Proximal, Wrist, Superstructure) {

        val targetHeight = 13.inch
        val intakeAxis by lazy { { Controls.operatorJoy.getRawAxis(1) } }
        val endCommand by lazy { { Controls.operatorJoy.getRawButton(12) } }
        //        var hasIncreasedStiltVelocity = false
        var startTime = 0.0
        val profile = TrapezoidProfile(TrapezoidProfile.Constraints(0.3, 3.0), // meters per sec and meters per sec ^2
                TrapezoidProfile.State(25.inch.meter, 0.0), TrapezoidProfile.State(12.inch.meter, 0.0)
        )

        override fun initialize() {
            stiltMotor.controller.setOutputRange(-0.80, 0.80)
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
            Proximal.wantedState = WantedState.Position((-15).degree)
            Elevator.setClimbMode()
            Proximal.setClimbPositionMode()
            Wrist.wantedState = WantedState.Position(88.degree)
            Controls.isClimbing = true
            startTime = Timer.getFPGATimestamp()
            LEDs.wantedState = LEDs.State.Blink(0.15.second, Color(130, 24, 30))
        }
        override fun execute() {
            if(Elevator.currentState.position < 18.inch && Elevator.currentState.position > 13.inch) Proximal.wantedState = WantedState.Position((-37).degree)
            else if(Elevator.currentState.position < 12.5.inch) {
                Proximal.wantedState = WantedState.Position((-45).degree)
                Wrist.wantedState = WantedState.Position(86.degree)
            }
//            if(Elevator.currentState.position < 15.inch && !hasIncreasedStiltVelocity) {
////                stiltMotor.controller.setOutputRange(-0.9, 0.9)
//                Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 0.31)
//                hasIncreasedStiltVelocity = true
//            }

            // actually set the positions
//            stiltMotor.setPosition(7.5.inch)
//            Elevator.wantedState = WantedState.Position(12.inch)
            val t = Timer.getFPGATimestamp() - startTime
            if(!profile.isFinished(t))  {
                val newState = profile.calculate(t)
                setClimbProfile(newState)
                Elevator.setClimbProfile(newState)
            } else {
                Elevator.wantedState = WantedState.Position(profile.m_goal.position.meter)
            }

            var s3nd = intakeAxis() * -1.0
            if(s3nd < -0.1) s3nd = -0.2

            val wantedIntake = if(Timer.getFPGATimestamp() < startTime + 2.0) 1.0 else if(s3nd > 0.0) s3nd + 0.35 else s3nd

            intakeWheels.setDutyCycle(wantedIntake)
            DriveSubsystem.lowGear = true
            DriveSubsystem.tankDrive(s3nd / 5.0, s3nd / 5.0)

            println("${intakeWheels.drawnCurrent.amp}, ${stiltMotor.drawnCurrent.amp}")
//            println("Elevator pos ${Elevator.motor.encoder.position.inch} Prox pos ${Proximal.motor.encoder.position.degree} " +
//                    "Prox output ${Proximal.motor.master.talonSRX.motorOutputPercent} Hab climber pos ${stiltMotor.encoder.position.inch} " +
//                    "Hab climber amp ${stiltMotor.drawnCurrent} Hab climber volts ${stiltMotor.voltageOutput}")
        }
        override fun isFinished() = endCommand() //stiltMotor.encoder.position < targetHeight + 0.5.inch
        //                && Elevator.motor.encoder.position < targetHeight + 0.5.inch
        override fun end(interrupted: Boolean) {
            Elevator.setMotionMagicMode()
            Elevator.wantedState = WantedState.Position(24.inch)
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
            Proximal.setMotionMagicMode()
            Proximal.wantedState = WantedState.Position((-20).degree)
            intakeWheels.setNeutral()
            stiltMotor.controller.setOutputRange(-0.15, 0.15)
            stiltMotor.setPosition(24.inch)
            Controls.isClimbing = false
            LEDs.wantedState = LEDs.State.Solid(Color.GREEN)
            GlobalScope.launch {
                delay(1500)
                LEDs.wantedState = LEDs.State.Off
                delay(250)
                LEDs.wantedState = LEDs.State.Default
            }
        }
    }

    override fun lateInit() {
        SmartDashboard.putData("test move", sequential {
            +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 80.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 95.degree) // TODO check preset
        })

//        SmartDashboard.putData("closed loop climb", fullS3ndClimbCommand)
//
//        SmartDashboard.putData("straight out", Superstructure.kHatchMid)
//        SmartDashboard.putData("SketchyTest", SketchyTest())
    }

    val safeRange = (10.inch..33.inch)

    val intakeWheels = FalconSRX(45, DefaultNativeUnitModel).apply {
        outputInverted = true
        configCurrentLimit(true, FalconSRX.CurrentLimitConfig(20.amp, 0.5.second, 15.amp))
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