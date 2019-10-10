package misc

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.derived.feetPerSecond
import org.ghrobotics.lib.mathematics.units.milli
import org.ghrobotics.lib.mathematics.units.second
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import java.awt.Color
import java.awt.Font
import java.text.DecimalFormat
import frc.robot.auto.paths.TrajectoryFactory

class TrajectoryVelocityPlotTest {

    @Test
    fun test() {
        displayTrajectory(TrajectoryFactory.sideStartReversedToRocketFPrepare)
    }

    fun displayTrajectory(trajectory: TimedTrajectory<Pose2dWithCurvature>) {

        var currentTime = 0.second
        val deltaTime = 20.milli.second

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()

        val iterator = trajectory.iterator()
        while (!iterator.isDone) {
            currentTime += deltaTime
            val sample = iterator.advance(deltaTime).state
            val velocity = sample.velocity
            xList += currentTime.second
            yList += velocity.feetPerSecond
        }

        val fm = DecimalFormat("#.###").format(trajectory.lastInterpolant.second)

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

        chart.addSeries("Robot velocity", xList.toDoubleArray(), yList.toDoubleArray())

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }
}