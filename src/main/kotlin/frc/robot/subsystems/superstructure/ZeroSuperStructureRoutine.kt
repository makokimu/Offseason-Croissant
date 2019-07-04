package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.mathematics.units.Length

import com.ctre.phoenix.motorcontrol.ControlMode

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

class ZeroSuperStructureRoutine(private val mZeroHeight: Length = kZeroHeight) : FalconCommand(SuperStructure,
        Elevator, Proximal, Wrist) {

    private var mCurrentState: ZeroingState? = null

    override fun runsWhenDisabled() = true

    private enum class ZeroingState {
        IDLE, WAITING_FOR_TRIGGER, ZEROED
    }

    // Called just before this Command runs the first time
    override fun initialize() {
        mCurrentState = ZeroingState.IDLE

        SmartDashboard.putBoolean("Elevator zeroed", false)

        SmartDashboard.putBoolean("Proximal zeroed", false)

        SmartDashboard.putBoolean("Wrist zeroed", false)

        SmartDashboard.putData(this)
    }

    // Called repeatedly when this Command is scheduled to run
    override fun execute() {

        val limitTriggered = Elevator.limitSwitchTriggered

        println("limitTriggered $limitTriggered")

        SmartDashboard.putString("Zeroing state", mCurrentState!!.name)
        SmartDashboard.putBoolean("Elevator limit switch", limitTriggered)

        SmartDashboard.putNumber("prox sensor pos", (Proximal.master.talonSRX.sensorCollection.pulseWidthPosition % 2048).toDouble())
        SmartDashboard.putNumber("wrist sensor pos", (Wrist.master.talonSRX.sensorCollection.pulseWidthPosition % 2048).toDouble())
        SmartDashboard.putNumber("elevator sensor pos", (Elevator.master.talonSRX.sensorCollection.pulseWidthPosition % 2048).toDouble())

        if (!DriverStation.getInstance().isDisabled)
            return
        // switch to observe desired behavior

        if (mCurrentState == ZeroingState.IDLE) {
            // System.out.println("in idle state");
            // var limitTriggered = limitStatus;
            if (!limitTriggered) {
                mCurrentState = ZeroingState.WAITING_FOR_TRIGGER
                // System.out.println("limit switch is off, waiting for retrigger");
                // break;
            }
        } else if (mCurrentState == ZeroingState.WAITING_FOR_TRIGGER) {
            // System.out.println("waiting for trigger");
            // limitTriggered = limitStatus;
            if (limitTriggered) {
                // System.out.println("observing elevator zeroed");
                observeElevatorZero()
                mCurrentState = ZeroingState.ZEROED
                // break;
            }
        }
        // else return;
    }
    // }

    private fun observeElevatorZero() {

        Elevator.master.talonSRX.set(ControlMode.PercentOutput, 0.0)
        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral()

        SmartDashboard.putBoolean("Elevator zeroed", true)
        SmartDashboard.putBoolean("Proximal zeroed", true)
        SmartDashboard.putBoolean("Wrist zeroed", true)

        val proximal = Proximal
        val tickkkkks = (Proximal.master.talonSRX.sensorCollection.pulseWidthPosition % 2048) // * if(Proximal.master.talonSRX.sensorCollection.pulseWidthPosition > 0) 1 else -1
        val targetProximal_COMP = 500 // 1900
        val delta = (tickkkkks - targetProximal_COMP) * -1
        val startingAngleTicks = proximal.master.model.toNativeUnitPosition((-90).degree).value // .talonSRX.getTicks(RoundRotation2d.getDegree(-78))\
        proximal.master.talonSRX.selectedSensorPosition = (0.0 + startingAngleTicks - delta).toInt()

        val wrist = Wrist
        val wristStart = wrist.master.model.toNativeUnitPosition((-45).degree).value // .getTicks(RoundRotation2d.getDegree(-43 + 4 - 9)) as Int
        val targetWristComp = 1050 // 1500 + 150
        val correctionDelta = Proximal.master.talonSRX.sensorCollection.pulseWidthPosition % 2048 * if (Proximal.master.talonSRX.sensorCollection.pulseWidthPosition > 0) 1 else -1
        val deltaW = (correctionDelta - targetWristComp) * 1
        wrist.master.talonSRX.selectedSensorPosition = (deltaW + wristStart).toInt()

        Elevator.master.encoder.resetPosition(Elevator.master.model.toNativeUnitPosition(mZeroHeight).value)
    }

    // Make this return true when this Command no longer needs to run execute()
    override fun isFinished(): Boolean {
        return mCurrentState == ZeroingState.ZEROED || !DriverStation.getInstance().isDisabled
    }

    // Called once after isFinished returns true
    override fun end(interrupted: Boolean) {
//        Elevator.elevatorZeroed = !interrupted
        SmartDashboard.putString("Zeroing state", mCurrentState!!.name)
    }

    companion object {
        private val kZeroHeight = 21.5.inch
    }
}
