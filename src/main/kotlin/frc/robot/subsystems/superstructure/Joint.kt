package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import org.ghrobotics.lib.mathematics.units.ElectricCurrent
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import kotlin.math.roundToInt
import org.team5940.pantry.experimental.command.SendableSubsystemBase


/**
 * Construct a joint. The Motors passed in are assumed to already have
 * their remote feedback sensor set up. Furthermore the master is
 * assuemd to be index 0.
 */
class Joint<T : SIUnit<T>> (
        val motors : List<FalconSRX<T>>,
        val arbitraryFeedForward : (pos : T) -> Volt,
        val cruiseVel : Velocity<T>,
        val cruiseAccel : Acceleration<T>,
        val pidfGains : List<Double>,
        val minPosition : NativeUnit,
        val maxPosition : NativeUnit,
        val maxContCurrent : ElectricCurrent,
        val maxPeakCurrent : ElectricCurrent,
        val peakCurrentDuration : Time
) : SendableSubsystemBase() {

    val master = motors[0]

    var controlMode : ControlMode = ControlMode.MotionMagic

    var setpoint: T = master.sensorPosition

    override fun periodic() {
        master.set(
                controlMode, setpoint,
                DemandType.ArbitraryFeedForward, arbitraryFeedForward.invoke(master.sensorPosition).value / 12
        )
    }

    init {

        motors.forEach{

            it.motionCruiseVelocity = cruiseVel
            it.motionAcceleration = cruiseAccel

            it.kP = pidfGains[0]
            it.kI = pidfGains[1]
            it.kD = pidfGains[2]
            it.kF = pidfGains[3]

            it.configPeakCurrentLimit(maxPeakCurrent.amp.roundToInt())
            it.configPeakCurrentDuration(peakCurrentDuration.millisecond.roundToInt())
            it.configContinuousCurrentLimit(maxContCurrent.amp.roundToInt())
            it.enableCurrentLimit(true)
        }

        setpoint = master.sensorPosition

        master.softLimitForward = minPosition
        master.softLimitReverse = maxPosition
        master.softLimitForwardEnabled = true
        master.softLimitReverseEnabled = true


    }

    /**
     * Getting this will get the position [T] of the joint
     * Setting this will set the sensor position of the joint
     */
    var sensorPosition : T
        get() = master.sensorPosition
        set(newValue) {
            master.sensorPosition = newValue
        }

    fun getPosition() : T {
        return master.sensorPosition
    }

    fun setPosition(newPos : T) {
        master.sensorPosition = newPos
    }

}