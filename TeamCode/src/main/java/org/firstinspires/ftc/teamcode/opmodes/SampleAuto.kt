package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
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

    val driveBase = MecDriveBase(this)
    val vision = Vision(this)
    val follower = Follower(driveBase)

    val pathBuilder = PathBuilder() // path generated with https://pedro-path-generator.vercel.app/
        .addPath(
            BezierCurve(
                Point(startingPosX, startingPosY, Point.CARTESIAN),
                Point(endingPosX, endingPosY, Point.CARTESIAN),
            )
        )
        .setLinearHeadingInterpolation(startingHeading.toRadians(), endingHeading.toRadians())

    val tagProcessor = vision.aprilTag

    override fun runOpMode() {

        val path = pathBuilder.build()

        follower.initialize()
        follower.setStartingPose(Pose(startingPosX, startingPosY, startingHeading.toRadians()))

        waitForStart()

        if (opModeIsActive()) {

            follower.followPath(path)
            FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)

            if (tagProcessor.freshDetections.size > 0) log("Detected AprilTag, ID = ${tagProcessor.freshDetections.first().id}")
            for (tag in tagProcessor.detections) {
                telemetry.addLine("\nTag ${tagProcessor.detections.indexOf(tag)}:")
                telemetry.addData("Tag ID", tag.id)
                telemetry.addData("x", tag.ftcPose.x)
                telemetry.addData("y", tag.ftcPose.y)
                telemetry.addData("z", tag.ftcPose.z)
                telemetry.addData("roll", tag.ftcPose.roll)
                telemetry.addData("pitch", tag.ftcPose.pitch)
                telemetry.addData("yaw", tag.ftcPose.yaw)
            }
        }
    }

    @Config
    companion object {
        @JvmField var startingPosX = 176.0
        @JvmField var startingPosY = -244.0
        @JvmField var startingHeading = -90.0 // degrees
        @JvmField var endingPosX = 176.0
        @JvmField var endingPosY = -244.0
        @JvmField var endingHeading = 90.0 // degrees
    }
}
