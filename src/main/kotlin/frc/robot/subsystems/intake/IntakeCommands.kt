@file:Suppress("unused", "MemberVisibilityCanBePrivate")

package frc.robot.subsystems.intake

import frc.robot.lib.andThen
import org.team5940.pantry.exparimental.command.InstantCommand
import org.team5940.pantry.exparimental.command.WaitCommand
import frc.robot.Constants.IntakeConstants.deployTime
import org.ghrobotics.lib.utils.DoubleSource
import org.team5940.pantry.exparimental.command.RunCommand

object IntakeCommands {

    fun closeIntake(instance: Intake) = InstantCommand(Runnable { instance.wantsOpen = false }) andThen WaitCommand(deployTime.second)
    fun openIntake(instance: Intake) = InstantCommand(Runnable { instance.wantsOpen = true }) andThen WaitCommand(deployTime.second)

    fun runIntake(cargoSpeed: DoubleSource, hatchSpeed: DoubleSource, instance: Intake) = RunCommand(Runnable {
        val isOpen = instance.wantsOpen
        val cargo = cargoSpeed.invoke()
        val hatch = hatchSpeed.invoke() * (if (!isOpen) -1 else 1)
//    if (!isOpen) hatch *= -1
    })



}