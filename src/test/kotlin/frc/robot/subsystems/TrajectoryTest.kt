package frc.robot.subsystems

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import frc.robot.Constants
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.auto.paths.asWaypoint
import frc.robot.subsystems.DriveConstants.kBeta
import frc.robot.subsystems.DriveConstants.kZeta
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.junit.Test

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.AngularAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.simulation.SimDifferentialDrive
import org.ghrobotics.lib.simulation.SimFalconMotor
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import java.awt.Color
import java.awt.Font
import java.text.DecimalFormat
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.pow


class TrajectoryTest {

    @Test
    fun test() {
        val rocketFPrepareToLoadingStation = TrajectoryFactory.generateTrajectory(
                    false,
                    listOf(
                            Pose2d(0.feet, 0.feet, 0.degree).asWaypoint(),
                            Pose2d(25.feet, 0.feet, 0.degree).asWaypoint()
                    ),
                    TrajectoryFactory.getConstraints(false, Pose2d(100.feet, 100.feet, 0.degree)), TrajectoryFactory.kMaxVelocity, TrajectoryFactory.kMaxAcceleration, TrajectoryFactory.kMaxVoltage
            )

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()

        val iterator = rocketFPrepareToLoadingStation.iterator()
        var max = 0.feet.velocity
        while(!iterator.isDone) {
            val state = iterator.advance(1.0.second.div(50.0))
            max = state.state.velocity.value.coerceAtLeast(max.value).meter.velocity
            xList.add(state.state.t.second)
            yList.add(state.state.velocity.feetPerSecond)
        }

        val fm = DecimalFormat("#.###").format(TrajectoryGeneratorTest.trajectory.lastInterpolant.second)

        val chart = XYChartBuilder().width(1800).height(1520).title("$fm seconds.")
                .xAxisTitle("X").yAxisTitle("Y").build()

        chart.styler.markerSize = 8
        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))

        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
        chart.styler.chartTitlePadding = 15

        chart.styler.xAxisMin = 0.0
        chart.styler.xAxisMax = rocketFPrepareToLoadingStation.lastInterpolant.second
        chart.styler.yAxisMin = 0.0
        chart.styler.yAxisMax = max.feetPerSecond + 1

        chart.styler.chartFontColor = Color.WHITE
        chart.styler.axisTickLabelsColor = Color.WHITE

        chart.styler.legendBackgroundColor = Color.GRAY

        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        chart.styler.plotGridLinesColor = Color.GRAY
        chart.styler.chartBackgroundColor = Color.DARK_GRAY
        chart.styler.plotBackgroundColor = Color.DARK_GRAY

        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)


    }

}


class RamseteControllerTest {

    private val kBeta = 2.0
    private val kZeta = 0.85

    @Test
    fun testTrajectoryFollower() {
        val ramseteTracker = RamseteTracker(
                kBeta,
                kZeta
        )

        val drive = SimDifferentialDrive(
                TrajectoryGeneratorTest.drive,
                SimFalconMotor(),
                SimFalconMotor(),
                ramseteTracker,
                1.05
        )

        var currentTime = 0.second
        val deltaTime = 20.milli.second

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()

        val refXList = arrayListOf<Double>()
        val refYList = arrayListOf<Double>()

        ramseteTracker.reset(TrajectoryGeneratorTest.trajectory)

        drive.robotPosition = ramseteTracker.referencePoint!!.state.state.pose
                .transformBy(Pose2d(-5.feet, 50.inch, 5.degree))

        var maxVel = 0.feet.velocity

        while (!ramseteTracker.isFinished //&&
                //(drive.robotPosition.translation.minus(
//                        TrajectoryGeneratorTest.trajectory.lastState.state.pose.translation)).norm.feet > 0.1
        ) {
            currentTime += deltaTime
            val output = (ramseteTracker.nextState(drive.robotPosition, currentTime))

//            println( (drive.robotPosition.translation.minus(
//                    drive.trajectoryTracker.referencePoint!!.state.state.pose.translation)).norm.feet)

            drive.setOutput(output)

            drive.update(deltaTime)

            val refVel = drive.trajectoryTracker.referencePoint!!.state.velocity
            maxVel = max(maxVel.value, refVel.value).meter.velocity

            xList += drive.robotPosition.translation.x.feet
            yList += drive.robotPosition.translation.y.feet

            val referenceTranslation = ramseteTracker.referencePoint!!.state.state.pose.translation
            refXList += referenceTranslation.x.feet
            refYList += referenceTranslation.y.feet

        }
        println(maxVel)
        val fm = DecimalFormat("#.###").format(TrajectoryGeneratorTest.trajectory.lastInterpolant.second)

        val chart = XYChartBuilder().width(1800).height(1520).title("$fm seconds.")
                .xAxisTitle("X").yAxisTitle("Y").build()

        chart.styler.markerSize = 8
        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))

        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
        chart.styler.chartTitlePadding = 15

        chart.styler.xAxisMin = 1.0
        chart.styler.xAxisMax = 26.0
        chart.styler.yAxisMin = 1.0
        chart.styler.yAxisMax = 26.0

        chart.styler.chartFontColor = Color.WHITE
        chart.styler.axisTickLabelsColor = Color.WHITE

        chart.styler.legendBackgroundColor = Color.GRAY

        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        chart.styler.plotGridLinesColor = Color.GRAY
        chart.styler.chartBackgroundColor = Color.DARK_GRAY
        chart.styler.plotBackgroundColor = Color.DARK_GRAY

        chart.addSeries("Trajectory", refXList.toDoubleArray(), refYList.toDoubleArray())
        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())

        val terror =
                TrajectoryGeneratorTest.trajectory.lastState.state.pose.translation - drive.robotPosition.translation
        val rerror = TrajectoryGeneratorTest.trajectory.lastState.state.pose.rotation - drive.robotPosition.rotation

        System.out.printf("%n[Test] X Error: %3.3f, Y Error: %3.3f%n", terror.x.feet, terror.y.feet)

//        assert(terror.norm.value.also {
//            println("[Test] Norm of Translational Error: $it")
//        } < 0.50)
//        assert(rerror.degree.also {
//            println("[Test] Rotational Error: $it degrees")
//        } < 5.0)

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }
}


class TrajectoryGeneratorTest {

    companion object {
        val drive = Constants.DriveConstants.kLowGearDifferentialDrive

        private val kMaxCentripetalAcceleration = 40.feet.acceleration
        private val kMaxAcceleration = 20.feet.acceleration
        private val kMaxAngularAcceleration = 20.radian.acceleration
        private val kMaxVelocity = 20.feet.velocity

        private const val kTolerance = 0.1

        private val kSideStart = Pose2d(1.54.feet, 23.234167.feet, 180.degree)
        private val kNearScaleEmpty = Pose2d(23.7.feet, 20.2.feet, 160.degree)

//        val trajectory = DefaultTrajectoryGenerator.generateTrajectory(
//                listOf(
////                        kSideStart,
////                        kSideStart + Pose2d((-13).feet, 0.feet, 0.degree),
////                        kSideStart + Pose2d((-19.5).feet, 5.feet, (-90).degree),
//                        kSideStart + Pose2d((-19.5).feet, 14.feet, (-90).degree),
//                        kNearScaleEmpty.mirror
//                ),
//                listOf(
//                        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
//                        DifferentialDriveDynamicsConstraint(drive, 10.0.volt),
//                        AngularAccelerationConstraint(kMaxAngularAcceleration)
//                ),
//                0.0.feet.velocity,
//                0.0.feet.velocity,
//                kMaxVelocity,
//                kMaxAcceleration,
//                true
//        )
        val trajectory = TrajectoryFactory.rocketFPrepareToLoadingStation
    }
}