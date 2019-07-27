package org.team5940.pantry.lib

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import io.github.oblarg.oblog.Logger
import kotlinx.coroutines.*
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.utils.loopFrequency
import org.ghrobotics.lib.wrappers.FalconTimedRobot

abstract class FishyRobot : FalconTimedRobot() {

    val isEnabled
            get() = wrappedValue.isEnabled

    var subsystemUpdateList = arrayListOf<ConcurrentlyUpdatingComponent>()

    private suspend fun periodicUpdate() {

        SmartDashboard.putNumber("lastTry", Timer.getFPGATimestamp())

        val subsystems = synchronized(subsystemUpdateList) {
            subsystemUpdateList
        }

        subsystems.forEach {
            SmartDashboard.putNumber("lastTry2", Timer.getFPGATimestamp())
            it.updateState(); it.useState(); SmartDashboard.putNumber("lastupdatetime", Timer.getFPGATimestamp())
        }
    }

    lateinit var job: Job

    var lastRobotMode = Mode.DISABLED
        private set

//    lateinit var notifier: Notifier

    override fun robotInit() {

        job = updateScope.launch {
            loopFrequency(50 /* hertz */) {
                println("trying...")
                periodicUpdate()
            }
        }

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
//        runBlocking { periodicUpdate() }
        if(!job.isActive || job.isCancelled || job.isCompleted) println("reeee job isn't running")
        super.robotPeriodic()
    }

    enum class Mode {
        AUTONOMOUS,
        TELEOPERATED,
        DISABLED,
    }

    companion object {
        protected val updateScopee = CoroutineScope(newFixedThreadPoolContext(1, "SubsystemUpdate"))
    }
}