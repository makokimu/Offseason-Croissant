package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kElevatorRange
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.* // ktlint-disable no-wildcard-imports

/**
 * The (singleton) [ConcurrentFalconJoint] elevator of Croissant.
 */
object Elevator : ConcurrentFalconJoint<Meter, FalconSRX<Meter>>() {

    override val motor = object : MultiMotorTransmission<Meter, FalconSRX<Meter>>() {

        override val master: FalconSRX<Meter> = FalconSRX(ElevatorPorts.TALON_PORTS[0],
                ElevatorPorts.LENGTH_MODEL)

        override val followers: List<FalconSRX<*>> = listOf(
                FalconSRX(ElevatorPorts.TALON_PORTS[1], DefaultNativeUnitModel),
                FalconSRX(ElevatorPorts.TALON_PORTS[2], DefaultNativeUnitModel),
                FalconSRX(ElevatorPorts.TALON_PORTS[3], DefaultNativeUnitModel))

        init {
            master.outputInverted = MASTER_INVERTED
            master.voltageCompSaturation = 12.volt
            master.feedbackSensor = ElevatorPorts.SENSOR
            master.talonSRX.setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)

            followers.forEachIndexed {
                index, followerMotor ->
                followerMotor.follow(master)
                followers[index].talonSRX.setInverted(ElevatorPorts.FOLLOWER_INVERSION[index])
                master.voltageCompSaturation = 12.volt
            }

            // Config master settings and stuff
            master.talonSRX.configForwardSoftLimitThreshold(58500)
            master.talonSRX.configReverseSoftLimitThreshold(1300)
            master.talonSRX.configForwardSoftLimitEnable(true)
            master.talonSRX.configReverseSoftLimitEnable(false)
            master.talonSRX.configPeakOutputForward(1.0)
            master.talonSRX.configPeakOutputReverse(-1.0)

//            master.talonSRX.enableVoltageCompensation(true)
            master.voltageCompSaturation = 12.volt

            setClosedLoopGains()
        }

        override fun setClosedLoopGains() =
                setMotionMagicGains()

        /**
         * Configure the master talon for motion magic control
         */
        fun setMotionMagicGains() {
            // TODO also wrap the solenoid boi for the shifter?

            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((5500.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(10000)
            master.talonSRX.configMotionSCurveStrength(0)

            master.setClosedLoopGains(
                    0.45 * 1.2, 4.0, ff = 0.3
            )
        }
    }

    /**
     * Configure the motor for position closed loop and
     * disable motion profile
     */
    fun setPositionMode() = motor.run {
        setClosedLoopGains(0.17, 0.0, 0.0)
        useMotionProfileForPosition = false
    }

    fun setClimbMode() = motor.run {
        setClosedLoopGains(0.375, 0.0, 0.0)
        useMotionProfileForPosition = false
    }

    /**
     * Configure the motor for motion profile closed loop
     * and enable the use of motion profile
     */
    fun setMotionMagicMode() = motor.run {
        setClosedLoopGains()
        useMotionProfileForPosition = true
    }

    // The elevator's limit switch
    private val innerStageMinLimitSwitch = DigitalInput(0)
    val limitSwitchTriggered: Boolean get() = !innerStageMinLimitSwitch.get()

    init { motor.encoder.position }

    // Set the elevator height to a sane-ish number by default
    override fun lateInit() { motor.encoder.resetPosition(30.0.inch) }

    /** The maximum distance by which the elevator setpoint can be offset */
    private val kMaxElevatorOffset = (-3.0).inch..3.0.inch

    var elevatorOffset: SIUnit<Meter> = 0.0.meter
        set(newValue) {
            field = newValue.coerceIn(kMaxElevatorOffset)
        }

    @Suppress("UNCHECKED_CAST")
    override fun customizeWantedState(wantedState: WantedState): WantedState =
            when (wantedState) {
                /** add the [wantedState] and [elevatorOffset] to get an offset total and then bound to the [kElevatorRange] */
                is WantedState.Position<*> -> { ((wantedState as WantedState.Position<Meter>) + elevatorOffset).coerceIn(kElevatorRange) }
                else -> wantedState
            }

    // The current state of the elevator
    override val currentState get() =
        motor.currentState

    /** Calculate the arbitrary feed forward given a [currentState] */
    override fun calculateFeedForward(currentState: MultiMotorTransmission.State<Meter>) =
            if (currentState.position > 33.0.inch) 1.2.volt else (-0.72).volt // volts
}
