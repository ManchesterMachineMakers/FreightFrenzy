package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Vision

@Autonomous(name = "April Tag Test")
class AprilTagTestAuto: LinearOpMode() {

    override fun runOpMode() {
        waitForStart()
        val vision = Vision(this)

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Distance to April Tag", vision.aprilTagDistance(13))
                idle()
            }
        }
    }
}