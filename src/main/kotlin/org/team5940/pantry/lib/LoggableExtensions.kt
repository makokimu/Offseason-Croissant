package org.team5940.pantry.lib

import edu.wpi.first.wpilibj.experimental.command.Subsystem
import io.github.oblarg.oblog.Loggable
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem

abstract class LoggableFalconSubsystem : FalconSubsystem(), Loggable

abstract class LoggableFalconCommand(vararg requirements: Subsystem) : FalconCommand(*requirements), Loggable