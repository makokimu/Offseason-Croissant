package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.PrintCommand
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.superstructure.SuperStructure.getDumbWrist
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.UnboundedRotation
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.radian
import org.team5940.pantry.lib.SIRotationConstants.kDegreesToRadians
import org.team5940.pantry.lib.SIRotationConstants.kRadianToDegrees
import kotlin.math.abs
import kotlin.math.min

class SyncedMove(goalAngle: Double, proximalMaxVel: Double, wristMaxVel: Double, private val isFrontToBack: Boolean) : FalconCommand(
        SuperStructure, Proximal, Wrist
) {

    private val goal: Double = goalAngle + if (isFrontToBack) -30 else 0
    private val goalRotation2d = (goal).radian
    private val goalWristRotation2d = (goalAngle).radian
    private var lastCommandedProximal: UnboundedRotation? = null
    private val proximalVelocity = abs(min(abs(proximalMaxVel), abs(wristMaxVel))) * 0.8 * (if (isFrontToBack) -1 else 1).toDouble()
    private var lastTime = 0.0
    private var moveIteratorFinished = false
    private var lastObservedState = SuperStructure.State.Position()

    constructor(goalAngle: Double, isFrontToBack: Boolean) : this(goalAngle, kProximalMaxVel, kWristMaxVel, isFrontToBack)

    override fun initialize() {
        lastCommandedProximal = SuperStructure.currentState.proximal.radian
        lastTime = Timer.getFPGATimestamp()
        moveIteratorFinished = false

        println("initial proximal: " + lastCommandedProximal!!)
    }

    override fun execute() {

        println("MOVE I9TERATOR $moveIteratorFinished")

        if (moveIteratorFinished)
            return

        val now = Timer.getFPGATimestamp()
        val dt = now - lastTime
        val currentState = SuperStructure.currentState
        this.lastObservedState = currentState

        var nextProximal = lastCommandedProximal

        if (Math.abs(SuperStructure.getUnDumbWrist(lastObservedState.wrist, lastObservedState.proximal) * kRadianToDegrees - lastCommandedProximal!!.degree) < 50) {
            nextProximal = this.lastCommandedProximal!!.plus((proximalVelocity * dt).radian)
        }

        if (nextProximal!!.degree < -205) {
            nextProximal = (-205).degree
            moveIteratorFinished = true
            println("SETTING MOVE ITERATOR TO TRUE 1")
        } else if (nextProximal.degree > 5) {
            nextProximal = (5).degree
            moveIteratorFinished = true
            println("SETTING MOVE ITERATOR TO TRUE 2")
        }

        println("next proximal: " + nextProximal.degree)

        if (isFrontToBack) {
            if (nextProximal.radian < goal)
                nextProximal = goalRotation2d
        } else {
            if (nextProximal.radian > goal)
                nextProximal = goalRotation2d
        }

        if (isFrontToBack) {
            if (nextProximal.radian < goal) {
                this.moveIteratorFinished = true
                println("SETTING MOVE ITERATOR TO TRUE 3")
            }
        } else if (nextProximal.radian > goal) {
            this.moveIteratorFinished = true
            println("SETTING MOVE ITERATOR TO TRUE 4")
        }

        val nextWrist = getDumbWrist(currentState.proximal, currentState.proximal)

        println("next elbow $nextProximal")
        println("next wrist $nextWrist")

        Proximal.wantedState = Proximal.WantedState.Position(nextProximal.radian)
        Wrist.wantedState = Wrist.WantedState.Position(nextWrist)

        this.lastCommandedProximal = nextProximal
        lastTime = now

        println("====================")

        println("last observed prox " + lastObservedState.proximal*kRadianToDegrees + " last wrist " +
                lastObservedState.wrist*kRadianToDegrees + " next elbow " + nextProximal +
                " next wrist " + nextWrist)

        println("====================")
    }

    override fun isFinished(): Boolean {
        val toReturn = (Wrist.isWithTolerance(5.0 * kDegreesToRadians) &&
                Proximal.isWithTolerance(5.0)) ||
                moveIteratorFinished

        println("pass thru done? $toReturn")
        println("moveIteratorFinished? $moveIteratorFinished")

        return toReturn
    }

    companion object {
        private const val kProximalMaxVel = 190.0 / 360.0 * 2.0 * Math.PI / 0.85
        private const val kWristMaxVel = kProximalMaxVel // 190d / 360d * 2 * Math.PI;

        val frontToBack
            get() = sequential {
                +PrintCommand("passiing thru front to back")
                +InstantCommand(Runnable { Intake.wantsOpen = false }, Intake)
                +ClosedLoopElevatorMove(22.5*SILengthConstants.kInchToMeter)
                +SyncedMove(-160 * kDegreesToRadians, true)
                +parallel {
                    +ClosedLoopProximalMove(-193.0 * kDegreesToRadians)
                    +ClosedLoopWristMove(-112.0 * kDegreesToRadians)
                }
                +ClosedLoopElevatorMove(5.5 * SILengthConstants.kInchToMeter)
            }

        val backToFront
            get() = sequential {
                +PrintCommand("passiing thru back to front")
                +InstantCommand(Runnable { Intake.wantsOpen = false }, Intake)
                +ClosedLoopElevatorMove(22.5*SILengthConstants.kInchToMeter)
                +SyncedMove(0.0 * kDegreesToRadians, false)
                +parallel {
                    +ClosedLoopProximalMove(0.0 * kDegreesToRadians)
                    +ClosedLoopWristMove(0.0 * kDegreesToRadians)
                    +ClosedLoopElevatorMove(15.0 * SILengthConstants.kInchToMeter)
                }
            }

        val shortPassthrough
            get() = sequential {
                +ClosedLoopElevatorMove(22.5*SILengthConstants.kInchToMeter)
                +SyncedMove(0.0 * kDegreesToRadians, false) }
    }
}