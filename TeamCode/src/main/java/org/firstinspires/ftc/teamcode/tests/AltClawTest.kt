package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Disabled
@TeleOp(name = "Alternate Claw Test")
class AltClawTest: LinearOpMode() {

    override fun runOpMode() {
        val altClaw = hardwareMap.servo.get(TODO())

        altClaw.scaleRange(TODO(), TODO())

        waitForStart()

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                when {
                    gamepad1.dpad_down -> altClaw.position = 1.0
                    gamepad1.dpad_down -> altClaw.position = 0.0
                }
            }
        }
    }
}