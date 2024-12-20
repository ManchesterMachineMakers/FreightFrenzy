package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.DashOpMode

@Autonomous(name = "April Tag Test")
class AprilTagTestAuto: LinearOpMode(), DashOpMode {

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val vision = Vision(this)

        waitForStart()
        FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                val robotPose = vision.robotPose()

                telemetry.addData("Robot X", robotPose?.position?.x)
                telemetry.addData("Robot Y", robotPose?.position?.y)
                telemetry.addData("Robot Z", robotPose?.position?.z)
                telemetry.addData("Robot Roll", robotPose?.orientation?.roll)
                telemetry.addData("Robot Pitch", robotPose?.orientation?.pitch)
                telemetry.addData("Robot Yaw", robotPose?.orientation?.yaw)

                telemetry.update()
                idle()
            }
        }
    }
}