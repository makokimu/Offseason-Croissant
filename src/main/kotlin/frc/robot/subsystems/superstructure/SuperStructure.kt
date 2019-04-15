package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import org.team5940.pantry.experimental.command.SendableSubsystemBase
import kotlin.math.roundToInt
import org.ghrobotics.lib.utils.Source
import java.util.function.Function

class SuperStructure {


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
                    DemandType.ArbitraryFeedForward, arbitraryFeedForward.invoke(master.sensorPosition).volt / 12
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

                setpoint = master.sensorPosition

            }

        }



        fun getPosition() : T {
            return master.sensorPosition
        }

        fun setPosition(newPos : T) {
            master.sensorPosition = newPos
        }

    }

}