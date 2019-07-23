package org.team5940.pantry.lib

import io.github.oblarg.oblog.Logger
import kotlinx.coroutines.*
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.wrappers.FalconTimedRobot

abstract class FishyRobot : FalconTimedRobot() {

    val isEnabled
            get() = wrappedValue.isEnabled

    var subsystemUpdateList = arrayListOf<ConcurrentlyUpdatingComponent>()

    private suspend fun periodicUpdate() {
        val subsystems = synchronized(subsystemUpdateList) {
            subsystemUpdateList
        }
        coroutineScope {
            subsystems.forEach {
                launch {
                    it.updateState(); it.useState()
                }
            }
        }
    }

    var lastRobotMode = Mode.DISABLED
        private set

    override fun robotInit() {
        updateScope.launchFrequency { periodicUpdate() }
        Logger.configureLoggingAndConfig(this, false)
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
        Logger.updateEntries()
        super.robotPeriodic()
    }

    enum class Mode {
        AUTONOMOUS,
        TELEOPERATED,
        DISABLED,
    }

    companion object {
        protected val updateScope = CoroutineScope(newFixedThreadPoolContext(2, "SubsystemUpdate"))
    }
}