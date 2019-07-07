package org.team5940.pantry.lib

import edu.wpi.first.wpilibj.Notifier
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import java.lang.Exception

abstract class FishyRobot : FalconTimedRobot() {

    private val stateUpdater = /*Notifier*/ {
        synchronized(subsystemUpdateList) {
            subsystemUpdateList.forEach {
                try {
                    it.updateState()
                }catch (e: Exception) {
                    e.printStackTrace()
                }
            }
        } }
        @Synchronized get
    private val stateUser = /*Notifier*/ {
        synchronized(subsystemUpdateList) {
            subsystemUpdateList.forEach {
                try {
                    it.useState()
                }catch (e: Exception) {
                    e.printStackTrace()
                }
            }
        } }
        @Synchronized get

    val isEnabled
            get() = wrappedValue.isEnabled

    internal val updateNotifier = Notifier {
        stateUpdater(); stateUser()
    }

    var subsystemUpdateList = arrayListOf<ConcurrentlyUpdatingComponent>()
        @Synchronized get
        @Synchronized set

    var lastRobotMode = Mode.DISABLED
        private set

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

    enum class Mode {
        AUTONOMOUS,
        TELEOPERATED,
        DISABLED,
    }

}