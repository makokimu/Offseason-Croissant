package frc.robot.subsystems.superstructure

import com.ctre.phoenix.CANifier
import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice
import com.ctre.phoenix.motorcontrol.RemoteSensorSource
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.ConcurrentFalconJoint
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.asPWMSource

object Wrist : ConcurrentFalconJoint<Radian, FalconSRX<Radian>>() {

    fun resetPosition(ticks: Int) {
        val encoder = synchronized(motor) { motor.encoder }
        encoder.resetPositionRaw(ticks.toDouble().nativeUnits)
    }

    private val canifier = CANifier(34)
    private val absoluteEncoder = canifier.asPWMSource(0.0 to 0.degree, 1.0 to 90.degree,
            CANifier.PWMChannel.PWMChannel0)

    fun zero() = motor.encoder.resetPosition(absoluteEncoder())

    override val motor = object : MultiMotorTransmission<Radian, FalconSRX<Radian>>() {
        override val master: FalconSRX<Radian> = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
                Ports.SuperStructurePorts.WristPorts.ROTATION_MODEL).apply { talonSRX.apply {

            var errorCode = configRemoteFeedbackFilter(canifier.deviceID, RemoteSensorSource.CANifier_Quadrature, 0, 100)

            if(errorCode != ErrorCode.OK)
                DriverStation.reportError("Could not set proximal remote sensor!!:  $errorCode", false)

            errorCode = configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 100)

            if(errorCode != ErrorCode.OK)
                DriverStation.reportError("Could not set proximal remote feedback sensor!! $errorCode", false)
        } }

        init {
            master.outputInverted = Ports.SuperStructurePorts.WristPorts.TALON_INVERTED
            master.feedbackSensor = Ports.SuperStructurePorts.WristPorts.SENSOR
            master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.WristPorts.TALON_SENSOR_PHASE)

            setClosedLoopGains()
        }

        override fun setClosedLoopGains() = setMotionMagicGains()

        fun setMotionMagicGains() {
            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((2000.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(3500)
            master.talonSRX.configMotionSCurveStrength(0)

            master.setClosedLoopGains(
                    3.5, 0.0, ff = 0.4
            )
        }
    }

    @Suppress("UNREACHABLE_CODE")
    fun setPositionMode() = motor.run {
        setClosedLoopGains(0.5, 0.0, 0.0)
        useMotionProfileForPosition = false
    }
    fun setMotionMagicMode() = motor.run {
        setClosedLoopGains()
        useMotionProfileForPosition = true
    }
}