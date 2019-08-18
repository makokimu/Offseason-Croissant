package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kProximalCos
import frc.robot.Constants.SuperStructureConstants.kProximalStatic
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.* // ktlint-disable no-wildcard-imports
import kotlin.math.cos
import kotlin.math.withSign

object Proximal : ConcurrentFalconJoint<Radian, FalconSRX<Radian>>() {

    fun resetPosition(ticks: Int) {
        val encoder = synchronized(motor) { motor.encoder }
        encoder.resetPositionRaw(ticks.nativeUnits)
    }

    override val motor = object : MultiMotorTransmission<Radian, FalconSRX<Radian>>() {

        override val master: FalconSRX<Radian> = FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[0],
                Ports.SuperStructurePorts.ProximalPorts.ROTATION_MODEL)

        override val followers: List<FalconSRX<*>> = listOf(
                FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[1], DefaultNativeUnitModel))

        init {
            if (Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION.size < followers.size)
                throw ArrayIndexOutOfBoundsException("Follower inversion list size contains less indices than the number of followers!")

            master.outputInverted = Ports.SuperStructurePorts.ProximalPorts.TALON_INVERTED
            master.feedbackSensor = Ports.SuperStructurePorts.ProximalPorts.SENSOR
            master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.ProximalPorts.TALON_SENSOR_PHASE)

            followers.forEachIndexed {
                index, followerMotor ->

                followerMotor.follow(master)
                followers[index].talonSRX.setInverted(Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION[index])
            }
            setClosedLoopGains()

            master.talonSRX.configForwardSoftLimitEnable(false)
            master.talonSRX.configReverseSoftLimitEnable(false)
        }

        override fun setClosedLoopGains() {
            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((1785.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(5000)
            master.talonSRX.configMotionSCurveStrength(0)

            master.setClosedLoopGains(
                    0.85, 6.0, ff = 0.45
            )
        }
    }

    override fun calculateFeedForward(currentState: MultiMotorTransmission.State<Radian>) =
            kProximalStatic.withSign(currentState.velocity.value) + (kProximalCos * cos(currentState.position).value)
}

fun <K : SIKey> SIUnit<K>.withSign(sign: Number) = SIUnit<K>(this.value.withSign(sign.toDouble()))

fun <K : SIKey> cos(x: SIUnit<K>) = SIUnit<K>(cos(x.value))
