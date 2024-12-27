package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.Subassembly

class AltClaw(opMode: OpMode) : Subassembly(opMode, "Alt Claw") {

    val clawServo = ReleaseServo(hardwareMap.servo.get("alt_claw"), Pair(0.1, 0.8))
    val rotateServo = hardwareMap.servo.get("alt_rotate")

    init {
//        clawServo.direction = Servo.Direction.REVERSE
//        rotateServo.direction = Servo.Direction.REVERSE
    }

    fun control(gamepad: Gamepad) {
        if (gamepad.a) clawServo.open()
        if (gamepad.y) clawServo.close()

        rotateServo.position += gamepad.right_stick_y * rotateServoCoefficient
    }

    companion object {
        @JvmField var rotateServoCoefficient = 0.003 // this value should be the highest possible without the pinion overshooting it's controls
    }
}