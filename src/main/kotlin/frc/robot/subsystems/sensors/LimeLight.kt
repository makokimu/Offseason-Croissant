package frc.robot.subsystems.sensors

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import frc.robot.Robot
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.milli
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToFalse
import org.ghrobotics.lib.utils.onChangeToTrue
import org.team5940.pantry.lib.Updatable
import kotlin.properties.Delegates

object LimeLight {

    val hasTarget get() = currentState.hasTarget

    private val currentStateMutex = Object()

    var currentState = State()
        get() = synchronized(currentStateMutex) { field }
        set(newValue) = synchronized(currentStateMutex) { field = newValue }

    var wantsLEDsOn by Delegates.observable(false) {_, _, wantsOn ->
        ledModeEntry.setDouble(if(wantsOn) 3.0 else 1.0)
    }

    var wantedPipeline by Delegates.observable(0.0) {_, _, newPipeline ->
        if(newPipeline < 0 || newPipeline > 9) {} else pipelineEntry.setDouble(newPipeline)
    }

    var wantsDriverMode by Delegates.observable(false) {_, _, wantsHighExposure ->
        camModeEntry.setDouble(if(wantsHighExposure) 1.0 else 0.0)
    }

    var wantedStreamMode by Delegates.observable(2.0) {_, _, newMode ->
        streamEntry.setDouble(newMode)
    }

    private val table = NetworkTableInstance.getDefault().getTable("limelight")
    private val tvEntry = table.getEntry("tv")
    private val txEntry = table.getEntry("tx")
    private val tyEntry = table.getEntry("ty")
    private val widthEntry = table.getEntry("tlong")
    private val heightEntry = table.getEntry("tshort")
    private val latencyEntry = table.getEntry("tl")
    private val ledModeEntry = table.getEntry("ledMode")
    private val camModeEntry = table.getEntry("camMode")
    private val pipelineEntry = table.getEntry("pipeline")
    private val streamEntry = table.getEntry("stream")

    data class State(
            val hasTarget: Boolean,
            val tx: SIUnit<Radian>,
            val ty: SIUnit<Radian>,
            val width: Double,
            val height: Double,
            val timestamp: SIUnit<Second>
    ) {
        constructor() : this(false, 0.degree, 0.degree, 0.0, 0.0, 0.second)
    }

    fun configureDisabled() {
        wantsLEDsOn = false
        wantsDriverMode = true
    }

    fun configureEnabled() {
        wantsLEDsOn = true
        wantsDriverMode = false
    }

    init {
        wantedStreamMode = 2.0
    }

    val enabledMonitor = { Robot.isEnabled }.monitor

    fun update() {
        enabledMonitor.onChangeToTrue { configureEnabled() }
        enabledMonitor.onChangeToFalse { configureDisabled() }

        val newState = State(
                tvEntry.getDouble(0.0) ==  1.0,
                -txEntry.getDouble(0.0).degree,
                tyEntry.getDouble(0.0).degree,
                widthEntry.getDouble(0.0),
                heightEntry.getDouble(0.0),
                latencyEntry.getDouble(0.0).milli.second + 11.milli.second + Timer.getFPGATimestamp().second
        )
        this.currentState = newState
    }

}