package org.team5940.pantry.lib

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.experimental.buttons.Button
import edu.wpi.first.wpilibj.experimental.buttons.JoystickButton

private enum class Buttons(val value : Int){
    kBumperLeft(5),
    kBumperRight(6),
    kStickLeft(9),
    kStickRight(10),
    kA(1),
    kB(2),
    kX(3),
    kY(4),
    kBack(7),
    kStart(8);
}



val XboxController.LeftBumperButton
    get() = JoystickButton(this, Buttons.kBumperLeft.value)


val XboxController.RightBumperButton : Button
    get() = JoystickButton(this, Buttons.kBumperRight.value)


val XboxController.LeftStickButton
    get() = JoystickButton(this, Buttons.kStickLeft.value)


val XboxController.RightStickButton
    get() = JoystickButton(this, Buttons.kStickRight.value)


val XboxController.AButtonButton
    get() = JoystickButton(this, Buttons.kA.value)


val XboxController.BButtonButton
    get() = JoystickButton(this, Buttons.kB.value)


val XboxController.XButtonButton
    get() = JoystickButton(this, Buttons.kX.value)


val XboxController.YButtonButton
    get() = JoystickButton(this, Buttons.kY.value)


val XboxController.BackButtonButton
    get() = JoystickButton(this, Buttons.kBack.value)


val XboxController.StartButtonButton
    get() = JoystickButton(this, Buttons.kStart.value)
