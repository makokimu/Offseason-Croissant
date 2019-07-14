package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.mathematics.units.Length

import com.ctre.phoenix.motorcontrol.ControlMode

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

class ZeroSuperStructureRoutine(private val mZeroHeight: Length = kZeroHeight) : FalconCommand(Superstructure,
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

//        println("limitTriggered $limitTriggered")

        SmartDashboard.putString("Zeroing state", mCurrentState!!.name)
        SmartDashboard.putBoolean("Elevator limit switch", limitTriggered)

        val positions = getPositions()

        SmartDashboard.putNumber("prox sensor pos", positions[0].toDouble())
        SmartDashboard.putNumber("wrist sensor pos", positions[1].toDouble())

        if (!DriverStation.getInstance().isDisabled)
            return

        if (mCurrentState == ZeroingState.IDLE) {
            if (!limitTriggered) {
                mCurrentState = ZeroingState.WAITING_FOR_TRIGGER
            }
        } else if (mCurrentState == ZeroingState.WAITING_FOR_TRIGGER) {
            if (limitTriggered) {
                observeElevatorZero(positions)
                mCurrentState = ZeroingState.ZEROED
            }
        }
    }

    private fun observeElevatorZero(positions: List<Int>) {

        Elevator.master.talonSRX.set(ControlMode.PercentOutput, 0.0)
        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral()

//        Wrist.master.talonSRX.configFeedbackNotContinuous()

        SmartDashboard.putBoolean("Elevator zeroed", true)
        SmartDashboard.putBoolean("Proximal zeroed", true)
        SmartDashboard.putBoolean("Wrist zeroed", true)

        val tickkkkks = positions[0]
        val targetProximal_COMP = 400 // 1900
        val delta = (tickkkkks - targetProximal_COMP) * -1 // whyyyy?
        val startingAngleTicks = Proximal.master.model.toNativeUnitPosition((-94).degree).value // .talonSRX.getTicks(RoundRotation2d.getDegree(-78))\
        Proximal.master.talonSRX.selectedSensorPosition = (0.0 + startingAngleTicks - delta).toInt()

        val wristStart = Wrist.master.model.toNativeUnitPosition((-45).degree).value // .getTicks(RoundRotation2d.getDegree(-43 + 4 - 9)) as Int
        val targetWristComp = 1050 // 1500 + 150
        val correctionDelta = positions[1]
        val deltaW = (correctionDelta - targetWristComp)
        Wrist.master.talonSRX.selectedSensorPosition = (deltaW + wristStart).toInt()

        Elevator.master.encoder.resetPosition(Elevator.master.model.toNativeUnitPosition(mZeroHeight).value)
    }

    fun getPositions(): List<Int> {

        val prox = (Proximal.master.talonSRX.sensorCollection.pulseWidthPosition % 4096).let {
            var temp = it
            while(it < 0) temp += 4096
            while(it > 4096) temp -= 4096
            temp
        }
        val wrist = (Wrist.master.talonSRX.sensorCollection.pulseWidthPosition % 4096).let {
            var temp = it
            while (it < 0) temp += 4096
            while (it > 4096) temp -= 4096
            temp
        }

        return listOf(prox, wrist)

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
        private val kZeroHeight = 33.inch // 21.5.inch, delta is 11.5in
    }
}
