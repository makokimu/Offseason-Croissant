package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import frc.robot.Controls
import frc.robot.subsystems.superstructure.Elevator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.wrappers.hid.getY
import org.team5940.pantry.lib.WantedState
import javax.naming.ldap.Control

class SyncronizedLiftClimbCommand : FalconCommand(ClimbSubsystem, Elevator) {

    var elevatorInPosition = false
    var stiltsInPosition = false

    override fun initialize() {
        ClimbSubsystem.stiltMotor.canSparkMax.setSmartCurrentLimit(5) // amps?
    }

    override fun execute() {
        // let's do voltage PID on the NEOOOOOOOOO
//        val elevatorPosition = WantedState.Position(Elevator.currentState.position)
        if(Elevator.currentState.position < 10.inch) elevatorInPosition = true
        if(!elevatorInPosition) Elevator.wantedState = WantedState.Voltage((-5).volt) else
            Elevator.wantedState = WantedState.Position(10.inch)
        ClimbSubsystem.wantedState = if(!stiltsInPosition) WantedState.Voltage((-5).volt) else WantedState.Position(10.inch)
    }

    override fun isFinished(): Boolean = elevatorInPosition and stiltsInPosition

}

class SketchyTest : FalconCommand(ClimbSubsystem, Elevator) {
    override fun isFinished() = false

    val elevatorVoltage by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft) }
    val stiltVoltage by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kRight) }

    override fun execute() {
        Elevator.wantedState = WantedState.Voltage(SIUnit(12.0 * elevatorVoltage()))
        ClimbSubsystem.wantedState = WantedState.Voltage(SIUnit(12.0 * stiltVoltage()))
    }
}