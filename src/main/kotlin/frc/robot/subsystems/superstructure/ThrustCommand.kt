package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.mathematics.threedim.geometry.Pose3d
import org.ghrobotics.lib.mathematics.threedim.geometry.Translation3d
import org.team5940.pantry.exparimental.command.SendableCommandBase

@Suppress("unused")
class ThrustCommand(
        var thrustDistance : Double, var thrustVelocity : Double, var superStructure: SuperStructure, var dt : Double = 20.0/1000.0
) : SendableCommandBase() {

    var initialState : SuperStructure.Preset = SuperStructure.hatchExtended
    var initialIntakeTransform : Translation3d = Translation3d(0.0,0.0,0.0)
    var goalIntakeTransform = initialIntakeTransform

    var trajectory : ArrayList<SuperStructure.Preset> = arrayListOf()

    init {
        addRequirements(superStructure)
    }

    override fun initialize() {
        initialState = /*superStructure.position*/ SuperStructure.Preset(10.0, 0.0, 0.0)
        initialIntakeTransform = /*superStructure.wrist.worldTransform.translation*/ Translation3d(22.0, Math.toRadians(-90.0), 0.0)
        goalIntakeTransform = initialIntakeTransform + Translation3d(thrustDistance, 0.0, 0.0)

        trajectory = generateThrustPlan(initialIntakeTransform, goalIntakeTransform, thrustDistance)

    }

    private fun generateThrustPlan(
            initialIntakeTransform: Translation3d, goalIntakeTransform: Translation3d, thrustDistance: Double
    ): ArrayList<SuperStructure.Preset> {

        // generate a list of linear points between the two states
        // assume that acceleration is infinite for now

        val deltaPos = goalIntakeTransform - initialIntakeTransform

        println("deltaPos: $deltaPos")

        val totalTime = thrustDistance / thrustVelocity

        val initialPreset = initialState

        val isProximalUnderCarriage = initialState.proximal < 0.0

        val path : ArrayList<SuperStructure.Preset> = arrayListOf()
        val samples : Double = (totalTime/dt)

        for(i in 0..samples.toInt()) {
            // iterate over all the poses we need to generate
            val progress : Double = i/samples

            println("progress $progress")

            println("distance to add ${(deltaPos / progress)}")

            val targetPose = initialIntakeTransform + (deltaPos / progress)

            println("target pose $targetPose")

            // iteratively solve the needed state reeEE
            path.add(superStructure.getPresetFromPose3d(initialPreset.wrist, targetPose, isProximalUnderCarriage, true))

        }

        return path

    }

}