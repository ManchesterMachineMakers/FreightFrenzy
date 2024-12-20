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
                val robotPose = vision.robotPose()

                telemetry.addData("Robot X", robotPose?.position?.x)
                telemetry.addData("Robot Y", robotPose?.position?.x)
                telemetry.addData("Robot Z", robotPose?.position?.x)
                telemetry.addData("Robot Roll", robotPose?.orientation?.roll)
                telemetry.addData("Robot Pitch", robotPose?.orientation?.pitch)
                telemetry.addData("Robot Yaw", robotPose?.orientation?.yaw)
                idle()
            }
        }
    }
}