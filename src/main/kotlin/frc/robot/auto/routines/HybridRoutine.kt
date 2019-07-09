package frc.robot.auto.routines

import frc.robot.auto.paths.TrajectoryFactory
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Time

class HybridRoutine(private val mode: Mode) : AutoRoutine() {

    enum class Mode(
        val path1: TimedTrajectory<Pose2dWithCurvature>,
        val path2: TimedTrajectory<Pose2dWithCurvature>,
        val path3: TimedTrajectory<Pose2dWithCurvature>,
        val isLeft: Boolean
    ) {
        LEFT(
            TrajectoryFactory.centerStartToCargoShipFL,
            TrajectoryFactory.cargoShipFLToLeftLoadingStation,
            TrajectoryFactory.loadingStationToRocketN.mirror(),
            true
        ),
        RIGHT(
            TrajectoryFactory.centerStartToCargoShipFR,
            TrajectoryFactory.cargoShipFRToRightLoadingStation,
            TrajectoryFactory.loadingStationToRocketN,
            false
        )
    }

    override val duration: Time
        get() = mode.path1.duration + mode.path2.duration + mode.path3.duration

    override val routine
        get() = sequential {
            TODO("not yet implemented")
        }
}