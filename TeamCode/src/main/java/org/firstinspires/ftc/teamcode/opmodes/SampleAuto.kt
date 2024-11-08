package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.toRadians

/*
The view is from the bottom of the robot looking upwards.
left on robot is y pos
front on robot is x pos
   /--------------\
   |     ____     |
   |     ----     |
   | ||        || |
   | ||        || |   left (y pos)
   |              |
   |              |
   \--------------/
     front (x pos)
 */

@Autonomous(name = "Sample Autonomous")
class SampleAuto: LinearOpMode(), DashOpMode {

    override fun runOpMode() {

        val driveBase = MecDriveBase(this)
        val follower = Follower(driveBase, hardwareMap)
        val vision = Vision(this)
        val telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        val pathBuilder = PathBuilder() // path generated with https://pedro-path-generator.vercel.app/
            .addPath(
                BezierCurve(
                    Point(startingPosX, startingPosY, Point.CARTESIAN),
                    Point(endingPosX, endingPosY, Point.CARTESIAN),
                )
            )
            .setLinearHeadingInterpolation(startingHeading.toRadians(), endingHeading.toRadians())

        val path = pathBuilder.build()
        val tagProcessor = vision.aprilTag

        follower.initialize()
        follower.setStartingPose(Pose(startingPosX, startingPosY, startingHeading.toRadians()))

        waitForStart()

        if (opModeIsActive()) {

            FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)
            follower.followPath(path)

            while (opModeIsActive()) {
//                if (tagProcessor.freshDetections != null && tagProcessor.freshDetections.size > 0) log("Detected AprilTag, ID = ${tagProcessor.freshDetections.first().id}")

                follower.update()
                for (tag in tagProcessor.detections) {
                    tag.ftcPose ?: break
                    telemetryA.addLine("\nTag ${tagProcessor.detections.indexOf(tag)}:")
                    telemetryA.addData("Tag ID", tag.id)
                    telemetryA.addData("x", tag.ftcPose.x)
                    telemetryA.addData("y", tag.ftcPose.y)
                    telemetryA.addData("z", tag.ftcPose.z)
                    telemetryA.addData("roll", tag.ftcPose.roll)
                    telemetryA.addData("pitch", tag.ftcPose.pitch)
                    telemetryA.addData("yaw", tag.ftcPose.yaw)
                    telemetryA.addData("bearing", tag.ftcPose.bearing)
                    telemetryA.addData("elevation", tag.ftcPose.elevation)
                }

                telemetryA.addData("\nacceleration", follower.acceleration)
                follower.telemetryDebug(telemetryA)
            }
        }
    }

    @Config
    companion object {
        @JvmField var startingPosX = 12.0
        @JvmField var startingPosY = 84.0
        @JvmField var startingHeading = 0.0 // degrees
        @JvmField var endingPosX = 36.0
        @JvmField var endingPosY = 84.0
        @JvmField var endingHeading = 180.0 // degrees
    }
}
