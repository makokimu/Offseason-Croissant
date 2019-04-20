package frc.robot

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.InvertType
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits

object Ports {

    const val kPCMID = 9

    object DrivePorts {
        val LEFT_PORTS = listOf(0, 1)
        val RIGHT_PORTS = listOf(2, 3)
        val SHIFTER_PORTS = listOf(4, 5)
    }

    object IntakePorts {
        val HATCH_PORT = 35
        val CARGO_PORT = 34
        val SHIFTER_PORTS = listOf(0, 1)
    }

    object SuperStructurePorts {
        object ElevatorPorts {
            val TALON_PORTS = listOf(21, 22, 23, 24)
            val MASTER_INVERTED = true
            val MASTER_SENSOR_PHASE = true
            val FOLLOWER_INVERSION = listOf(InvertType.OpposeMaster, InvertType.FollowMaster, InvertType.FollowMaster)
            val LENGTH_MODEL = NativeUnitLengthModel(4096.nativeUnits, 1.5.inch)
            val SENSOR = FeedbackDevice.CTRE_MagEncoder_Relative
        }
        object ProximalPorts {
            val TALON_PORT = 31
            val TALON_INVERTED = true
            val TALON_SENSOR_PHASE = false
            val ROTATION_MODEL = NativeUnitRotationModel(4096.nativeUnits)
            val SENSOR = FeedbackDevice.CTRE_MagEncoder_Absolute
        }
    }

}