//package frc.robot.auto.routines
//
//import frc.robot.auto.Autonomous
//import frc.robot.auto.paths.TrajectoryFactory
//import frc.robot.subsystems.superstructure.Superstructure
//import org.ghrobotics.lib.commands.sequential
//import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
//import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
//import org.ghrobotics.lib.utils.withEquals
//
//class TestTrajectoriesRoutine : AutoRoutine() {
//    private val path1 = TrajectoryFactory.sideStartToRocketF
//    private val path2 = TrajectoryFactory.rocketNToLoadingStation
//    private val path3 = TrajectoryFactory.loadingStationToRocketF
//    private val path4 = TrajectoryFactory.rocketFToDepot
//
//    override val duration: SIUnit<Second>
//        get() = path1.duration + path2.duration + path3.duration + path4.duration
//
//    override val routine
//        get() = sequential {
//            val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)
//
//            +Superstructure.kBackHatchFromLoadingStation.withTimeout(1.second)
//            +followVisionAssistedTrajectory(path1, pathMirrored, 8.feet)
////
////
////            +DriveSubsystem.followVisionAssistedTrajectory(path1, pathMirrored, 3.feet, 2.feet)
////            +DriveSubsystem.followTrajectory(path2, pathMirrored)
////            +DriveSubsystem.followVisionAssistedTrajectory(path3, pathMirrored, 4.feet, 3.feet)
//        }
//}
