package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.powerCurve
import kotlin.math.abs
import kotlin.math.max

class DriveBase(opMode: OpMode) : Subassembly(opMode, "Drive Base") {

    val leftFront = hardwareMap.dcMotor.get("left_front")
    val rightFront = hardwareMap.dcMotor.get("right_front")
    val leftRear = hardwareMap.dcMotor.get("left_rear")
    val rightRear = hardwareMap.dcMotor.get("right_rear")

    companion object {
        const val wheelBaseWidth = 420.0 // mm
        const val motorEncoderEventsPerRevolution = 537.7
        const val wheelCircumference = 310.0 // mm
        const val motorEncoderEventsPerMM = motorEncoderEventsPerRevolution / wheelCircumference
    }

    enum class TurnDirection {
        LEFT,
        RIGHT
    }

    init {
        // direction = FORWARD by default
        leftFront.direction = DcMotorSimple.Direction.REVERSE
//        rightFront.direction = DcMotorSimple.Direction.REVERSE
        leftRear.direction = DcMotorSimple.Direction.REVERSE
//        rightRear.direction = DcMotorSimple.Direction.REVERSE

        opMode.log("DriveBase successfully initialized")
    }

    fun opInit() {
        setRunMode(RunMode.RUN_WITHOUT_ENCODER)
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
    }

    fun control(gamepad: Gamepad) {
        // from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        val leftX: Double = -gamepad.left_stick_x.toDouble()
        val leftY: Double = gamepad.left_stick_y.toDouble()
        val rightX: Double = -gamepad.right_stick_x.toDouble()

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        val denominator = max(abs(leftY) + abs(leftX) + abs(rightX), 1.0)
        val leftFrontPower = (leftY + leftX + rightX)
        val rightFrontPower = (leftY - leftX - rightX)
        val leftRearPower = (leftY - leftX + rightX)
        val rightRearPower = (leftY + leftX - rightX)

        leftFront.power = powerCurve(leftFrontPower)
        rightFront.power = powerCurve(rightFrontPower)
        leftRear.power = powerCurve(leftRearPower)
        rightRear.power = powerCurve(rightRearPower)
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addLine()
    }

    data class MoveRobotCalculations(
            val x: Double,
            val y: Double,
            val yaw: Double,
            val leftFront: Double = x - y - yaw,
            val rightFront: Double = x + y + yaw,
            val leftRear: Double = (x + y - yaw),
            val rightRear: Double = x - y + yaw
    )

    /**
     * Taken from the RobotAutoDriveToAprilTagOmni example (2023) Move robot according to desired
     * axes motions
     *
     * Positive X is forward
     *
     * Positive Y is strafe left
     *
     * Positive Yaw is counter-clockwise
     */
    fun moveRobot(x: Double, y: Double, yaw: Double) {
        // Calculate wheel powers.
        var (_, _, _, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower) =
                MoveRobotCalculations(x, y, yaw)

        // Normalize wheel powers to be less than 1.0
        var max = max(abs(leftFrontPower), abs(rightFrontPower))
        max = max(max, abs(leftBackPower))
        max = max(max, abs(rightBackPower))
        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }

        // Send powers to the wheels.
        leftFront.power = leftFrontPower
        rightFront.power = rightFrontPower
        leftRear.power = leftBackPower
        rightRear.power = rightBackPower
    }

    val motors = listOf(leftFront, rightFront, leftRear, rightRear)
    fun List<DcMotor>.setMode(mode: RunMode) {
        map { it.mode = mode }
    }

    fun setMode(mode: RunMode) {
        motors.setMode(mode)
    }

    fun setTargetPositions(lf: Int, rf: Int, lr: Int, rr: Int) {
        leftFront.targetPosition = lf
        rightFront.targetPosition = rf
        leftRear.targetPosition = lr
        rightRear.targetPosition = rr
    }

    fun setPower(lf: Double, rf: Double, lr: Double, rr: Double) {
        leftFront.power = lf
        rightFront.power = rf
        leftRear.power = lr
        rightRear.power = rr
    }

    fun setPower(powers: Array<Double>) {
        motors.forEachIndexed { index, motor -> motor.power = powers[index] }
    }

    fun runToPosition(lf: Int, rf: Int, lr: Int, rr: Int) {
        setMode(RunMode.STOP_AND_RESET_ENCODER)
        setMode(RunMode.RUN_USING_ENCODER)
        setTargetPositions(abs(lf), abs(rf), abs(lr), abs(rr))
        motors.map { (it as DcMotorEx).targetPositionTolerance = 5 }
        setMode(RunMode.RUN_TO_POSITION)

        leftFront.power = if (lf < 0) -0.5 else 0.5
        rightFront.power = if (rf < 0) -0.5 else 0.5
        leftRear.power = if (lr < 0) -0.5 else 0.5
        rightRear.power = if (rr < 0) -0.5 else 0.5
    }

    fun runToPosition(ticks: Int) {
        runToPosition(ticks, ticks, ticks, ticks)
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param power the power to each motor (we can make this a list if needed)
     * @param encoderTicks a list of encoder tick values, distances that each motor should go by the
     * encoders.
     * @param tolerance the tolerance to set on each motor
     */
    fun go(power: Double, encoderTicks: Array<Int>, tolerance: Int): Double {
        RobotLog.i("Power: $power Encoder Ticks: $encoderTicks")
        RobotLog.i("Tolerance: $tolerance")

        // set each motor, based on drive configuration/travel direction
        for (i in motors.indices) {
            motors[i].mode = RunMode.RUN_USING_ENCODER
            motors[i].targetPosition =
                    encoderTicks[
                            i] // calcTargetPosition(motors[i].getDirection(), currentPositions[i],
            // encoderTicks[i]));
            (motors[i] as DcMotorEx).targetPositionTolerance = tolerance
            motors[i].mode = RunMode.RUN_TO_POSITION
            motors[i].power = power
        }
        return power
    }

    enum class TravelDirection { // see this link on enum naming convention: https://kotlinlang.org/docs/coding-conventions.html#names-for-backing-properties
        BASE,
        FORWARD,
        REVERSE,
        PIVOT_LEFT,
        PIVOT_RIGHT,
        STRAFE_LEFT,
        STRAFE_LEFT_FORWARD,
        STRAFE_LEFT_BACKWARD,
        STRAFE_RIGHT,
        STRAFE_RIGHT_FORWARD,
        STRAFE_RIGHT_BACKWARD,
        @Deprecated("Newer drive bases do not support pitching") PITCH
    }

    fun mecanumCoefficientsForDirection(direction: TravelDirection) =
            when (direction) {
                TravelDirection.BASE, TravelDirection.FORWARD -> arrayOf(1, 1, 1, 1)
                TravelDirection.REVERSE -> arrayOf(-1, -1, -1, -1)
                TravelDirection.PIVOT_LEFT -> arrayOf(-1, 1, -1, 1)
                TravelDirection.PIVOT_RIGHT -> arrayOf(1, -1, 1, -1)
                TravelDirection.STRAFE_LEFT -> arrayOf(-1, 1, 1, -1)
                TravelDirection.STRAFE_LEFT_FORWARD -> arrayOf(0, 1, 1, 0)
                TravelDirection.STRAFE_LEFT_BACKWARD -> arrayOf(-1, 0, 0, -1)
                TravelDirection.STRAFE_RIGHT -> arrayOf(1, -1, -1, 1)
                TravelDirection.STRAFE_RIGHT_FORWARD -> arrayOf(1, 0, 0, 1)
                TravelDirection.STRAFE_RIGHT_BACKWARD -> arrayOf(0, -1, -1, 0)
                TravelDirection.PITCH -> arrayOf(0, 0, 0, 0)
            }

    fun setRunMode(runMode: RunMode): Boolean {
        return try {
            for (motor in motors) {
                motor.mode = runMode
            }
            true
        } catch (e: Exception) {
            RobotLog.i(e.message)
            false
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior): Boolean {
        return try {
            for (motor in motors) {
                motor.zeroPowerBehavior = zeroPowerBehavior
            }
            true
        } catch (e: Exception) {
            RobotLog.i(e.message)
            false
        }
    }
}