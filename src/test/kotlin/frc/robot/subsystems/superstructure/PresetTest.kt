package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.mathematics.threedim.geometry.Pose3d
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.junit.Test

class PresetTest {

    @Test
    fun testPresetMath() {

        val elevator = Elevator.getMockElevator()
        val proximal = Joint.getMockJoint()
        val wrist = Joint.getMockJoint()

        val superStructure = SuperStructure(
                elevator, proximal, wrist
        )

        // x axis is left/right from a cross section view, y axis is elevator up/down
        val preset = superStructure.getPresetFromPose3d(0.degree.radian,
                Translation3d(30.inch.meter, 40.inch.meter, 0.0),
                true, true)

        // x axis is left/right from a cross section view, y axis is elevator up/down
        val preset2 = superStructure.getPresetFromPose3d(-45.degree.radian,
                Translation3d(11.3.inch.meter, 40.inch.meter, 0.0),
                true, true)

        // x axis is left/right from a cross section view, y axis is elevator up/down
        val preset3 = superStructure.getPresetFromPose3d(-45.degree.radian,
                Translation3d(0.inch.meter, 40.inch.meter, 0.0),
                true, true)

        // x axis is left/right from a cross section view, y axis is elevator up/down
        val preset4 = superStructure.getPresetFromPose3d(-45.degree.radian,
                Translation3d(500.inch.meter, 40.inch.meter, 0.0),
                true, true)


    }

}