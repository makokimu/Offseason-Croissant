package org.team5940.pantry.lib

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.experimental.command.CommandScheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.*
import kotlinx.coroutines.*
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.utils.loopFrequency
import org.ghrobotics.lib.wrappers.FalconTimedRobot

abstract class FishyRobot : FalconTimedRobot() {

    val isEnabled
            get() = wrappedValue.isEnabled

    private suspend fun periodicUpdate() {

        SmartDashboard.putNumber("lastTry", Timer.getFPGATimestamp())

        DriveSubsystem.updateState()
        DriveSubsystem.useState()
        Superstructure.updateState()
        Superstructure.useState()
    }

    val job = arrayListOf<Job>()

    var lastRobotMode = Mode.DISABLED
        private set

    override fun robotInit() {

        job.add(updateScope.launch {
            loopFrequency(75 /* hertz */) {
                periodicUpdate()
            }
        })

        job.add(updateScope.launch {
            loopFrequency(4) {
                SmartDashboard.putString("Joint states", Superstructure.currentState.asString())
            }
        })

        CommandScheduler.getInstance().onCommandInitialize { command -> println("[CommandScheduler] Command ${command.name} initialized!") }
//        CommandScheduler.getInstance().onCommandExecute { command -> println("[CommandScheduler] Command ${command.name} execute!") }
        CommandScheduler.getInstance().onCommandInterrupt { command -> println("[CommandScheduler] Command ${command.name} interrupted!") }
        CommandScheduler.getInstance().onCommandFinish { command -> println("[CommandScheduler] Command ${command.name} finished!!") }

        super.robotInit()
    }

    override fun disabledInit() {
        lastRobotMode = Mode.DISABLED
        super.disabledInit()
    }

    override fun autonomousInit() {
        lastRobotMode = Mode.AUTONOMOUS
        super.autonomousInit()
    }

    override fun teleopInit() {
        lastRobotMode = Mode.TELEOPERATED
        super.teleopInit()
    }

    override fun robotPeriodic() {
        updatableSubsystems.forEach { it.update() }
//        runBlocking { periodicUpdate() }
        super.robotPeriodic()
    }

    enum class Mode {
        AUTONOMOUS,
        TELEOPERATED,
        DISABLED,
    }

    private val updatableSubsystems = arrayListOf<Updatable>()

    operator fun Updatable.unaryPlus() {
        updatableSubsystems.add(this)
    }

    companion object {
        val updateScope = CoroutineScope(newFixedThreadPoolContext(1, "SubsystemUpdate"))
    }
}

interface Updatable { fun update() }