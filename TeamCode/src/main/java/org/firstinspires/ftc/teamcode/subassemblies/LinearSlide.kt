@file:Suppress("MemberVisibilityCanBePrivate")
package org.firstinspires.ftc.teamcode.subassemblies

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.Subassembly

class LinearSlide(opMode: OpMode): Subassembly(opMode, "Linear Slide") {

    val leftSlide = hardwareMap.dcMotor.get("left_slide") as DcMotorEx
    val rightSlide = hardwareMap.dcMotor.get("right_slide") as DcMotorEx
    val slides = arrayOf(leftSlide, rightSlide)
    val pinion = hardwareMap.servo.get("carter's_opinion")

    var slidePosition = 0.0
        set(value) {
            slides.forEach { it.targetPosition = (value * slideEncoderTicks / slideGearCircumference).toInt() }
            field = value
        }
        get() = slides.map { it.power }.average() / slideEncoderTicks * slideGearCircumference

    var slidePower = 0.0
        set(value) {
            slides.forEach { it.power = value }
            field = value
        }
        get() = slides.map { it.power }.average()

    var pinionPosition = 0.0
        set(value) {
            pinion.position = value * servoRangeOfMotion / pinionGearCircumference
            field = value
        }
        get() = pinion.position / servoRangeOfMotion * pinionGearCircumference

    var pinionPower = 0.0
        set(value) {
            pinion.position += (value * pinionSpeedCoefficient)
            field = value
        }

    init {
        leftSlide.direction = DcMotorSimple.Direction.REVERSE
//        rightSlide.direction = DcMotorSimple.Direction.REVERSE
//        pinion.direction = DcMotorSimple.Direction.REVERSE

        leftSlide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightSlide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    /**
     * Set the position of the linear slide and pinion
     * based on the x and y coordinates of the robot
     * the origin is at the bottom center of the robot
     *
     * @param x the target x coordinate with 0 being the center of the robot in CM
     * @param y the target y coordinate with 0 being the bottom of the robot in CM
     */
    fun setPosition(x: Double, y: Double) {
        slidePosition = x
        pinionPosition = y
    }

    fun control(x: Float, y: Float) {
        slidePower = y.toDouble()
        pinionPower = x.toDouble()
    }

    @Config
    companion object {
        @JvmField var slideEncoderTicks = 384.5 // PPR at output shaft
        @JvmField var slideGearDiameter = 4.0 // cm
        var slideGearCircumference = slideGearDiameter * Math.PI
        @JvmField var servoRangeOfMotion = 300.0 // degrees
        @JvmField var pinionGearDiameter = 2.0 // cm
        var pinionGearCircumference = pinionGearDiameter * Math.PI
        @JvmField var pinionSpeedCoefficient = 0.5 // this should be the highest value that cause the pinion to overshoot
    }
}