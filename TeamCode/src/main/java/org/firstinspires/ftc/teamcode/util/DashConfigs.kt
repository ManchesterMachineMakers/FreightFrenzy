@file:Suppress("unused", "MayBeConstant")
package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

@Config
object MotorDirections {
    @JvmField var LEFT_FRONT = DcMotorSimple.Direction.FORWARD
    @JvmField var RIGHT_FRONT = DcMotorSimple.Direction.REVERSE
    @JvmField var LEFT_REAR = DcMotorSimple.Direction.REVERSE
    @JvmField var RIGHT_REAR = DcMotorSimple.Direction.FORWARD
    @JvmField var ARM = DcMotorSimple.Direction.FORWARD
    @JvmField var WINCH = DcMotorSimple.Direction.FORWARD
}

@Config
object ServoDirections {
    @JvmField var DRONE_LAUNCHER = Servo.Direction.REVERSE
    @JvmField var LEFT_RELEASE = Servo.Direction.FORWARD
    @JvmField var RIGHT_RELEASE = Servo.Direction.REVERSE
    @JvmField var WRIST = Servo.Direction.REVERSE
}

@Config
object ServoRanges {
    @JvmField var DRONE_LAUNCHER = Pair(0.6, 0.72)
    @JvmField var LEFT_RELEASE = Pair(0.15, 0.4)
    @JvmField var RIGHT_RELEASE = Pair(0.4, 0.85)
    @JvmField var WRIST = Pair(0.25, 0.78)
}

@Config
object ShowTelemetry {
    @JvmField var JOYSTICK = false
    @JvmField var ARM = true
    @JvmField var DRIVE_BASE = false
    @JvmField var DRONE_LAUNCHER = false
    @JvmField var IMU_MANAGER = false
    @JvmField var PIXEL_RELEASES = true
    @JvmField var VISION = false
    @JvmField var WINCH = false
}
