package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog

class ReleaseServo(val servo: Servo, val scaleRange: Pair<Double, Double>) {

    var direction = Servo.Direction.FORWARD
    var isOpen = servo.position > 0.5
        private set

    init {
        servo.direction = direction
        servo.scaleRange(scaleRange.first, scaleRange.second)
    }

    fun open() {
        RobotLog.i("${this::class.simpleName} opened")
        servo.position = 1.0
        isOpen = true
    }
    fun close() {
        RobotLog.i("${this::class.simpleName} closed")
        servo.position = 0.0
        isOpen = false
    }
    fun toggle() {
        if (isOpen) close()
        else open()
        RobotLog.i("${this::class.simpleName} toggled, isOpen=$isOpen")
    }
}