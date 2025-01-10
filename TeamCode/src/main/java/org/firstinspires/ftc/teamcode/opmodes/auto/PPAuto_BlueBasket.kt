package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.subassemblies.AltClaw
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.DashOpMode


@Autonomous(name = "Blue Basket Autonomous")
class PPAuto_BlueBasket: LinearOpMode(), DashOpMode {
    val driveBase = MecDriveBase(this)
    val claw = AltClaw(this)
    val linearSlide = LinearSlide(this)
    val follower = Follower(driveBase, hardwareMap)
    val vision = Vision(this)
    val tagProcessor = vision.aprilTag
    val startingPose = Pose(0.000, 95.000, 0.0)
    var currentPose: Pose = startingPose
    val telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

    fun runPath(path: PathChain) {
        follower.followPath(path, true)
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
    override fun runOpMode() {

//        val driveBase = MecDriveBase(this)
//        val follower = Follower(driveBase, hardwareMap)
//        val vision = Vision(this)
//        val telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
//
//        val tagProcessor = vision.aprilTag

        follower.initialize()
        follower.setStartingPose( // Blue Alliance, Basket Side
//            Pose(0.000, 95.000,  SampleAuto.startingHeading.toRadians())
            startingPose
        )

        waitForStart()

        if (opModeIsActive()) {

            FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)
            currentPose = deliverPreload(linearSlide, claw, follower, startingPose)

            parkAndAscend(linearSlide, claw, follower, currentPose)

            // TODO: Make sure the arm is extended to touch the low rung on the submersible for ascension points.
        }
    }

//    val pathSegments: Queue<Pose> = LinkedList<Pose>(listOf(
//        Pose(35.537, 85.470, 180.0), // approach rungs to hang a preloaded specimen
//        Pose(40.182, 109.857, 245.0), // fetch the closest neutral sample from the floor
//        Pose(16.259, 127.973, 325.0), // deliver the neutral sample
//        Pose(33.911, 130.761, 180.0), // fetch the next neutral sample
//        Pose(16.259, 128.206, 325.0), // deliver it
//        Pose(37.163, 133.315, 225.0), // fetch the last neutral sample
//        Pose(16.492, 127.973, 325.0), // deliver it
//        Pose(59.924, 106.838, 0.0), // avoid the legs of the submersible
//        Pose(59.924, 93.831, 90.0) // park touching the low rung of the submersible
//    ))

    // paths generated with https://pedro-path-generator.vercel.app/
    fun deliverPreload(slide: LinearSlide, claw: AltClaw, follower: Follower, startPose: Pose): Pose {
        val p = PathBuilder()
            .addPath( // Line 1 - approach rungs to hang a preloaded specimen
                BezierLine(
                    // Point(0.000, 95.000, Point.CARTESIAN),
                    Point(startPose),
                    Point(35.537, 85.470, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(startPose.heading), Math.toRadians(180.0))
        runPath(p.build())
        slide.setPosition(130.0, 260.0)
        claw.rotateServo.position = 0.5
        slide.setPosition(130.0, 255.0)
        claw.clawServo.open()
        
        return follower.pose
    }

    fun deliverSample(slide: LinearSlide, claw: AltClaw, follower: Follower, startPose: Pose): Pose {
        val p = PathBuilder()
            .addPath( // Line 3 - deliver sample 3
                BezierLine(
                    Point(startPose),
                    Point(16.259, 127.973, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(startPose.heading), Math.toRadians(325.0))
        runPath(p.build())
        slide.setPosition(130.0, 300.0) // raise to the top basket
        claw.clawServo.open() // let the sample go
        slide.setPosition(50.0, 100.0) // lower the arm
        return follower.pose
    }

    // it would be neat if we could use the camera to help us fetch the samples, but it's
    // currently mounted on the wrong side of the robot.
    fun fetchNeutralSample3 (slide: LinearSlide, claw: AltClaw, follower: Follower, startPose: Pose): Pose {
        val p = PathBuilder()
            .addPath( // Line 2 - fetch the closest neutral sample from the floor
                BezierLine(
//                    Point(35.537, 85.470, Point.CARTESIAN),
                    Point(startPose),
                    Point(40.182, 109.857, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(245.0))
        runPath(p.build())
        return follower.pose
    }

    fun deliverSample3(slide: LinearSlide, claw: AltClaw, follower: Follower, startPose: Pose): Pose {
        val p = PathBuilder()
            .addPath( // Line 3 - deliver sample 3
                BezierLine(
//                    Point(40.182, 109.857, Point.CARTESIAN),
                    Point(startPose),
                    Point(16.259, 127.973, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(245.0), Math.toRadians(325.0))
        runPath(p.build())
        return follower.pose
    }

    fun fetchNeutralSample2(slide: LinearSlide, claw: AltClaw, follower: Follower, startPose: Pose): Pose {
        val p = PathBuilder()
            .addPath( // Line 4 - fetch the next neutral sample from the floor
                BezierLine(
//                    Point(16.259, 127.973, Point.CARTESIAN),
                    Point(startPose),
                    Point(33.911, 130.761, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(325.0), Math.toRadians(180.0))
        runPath(p.build())
        return follower.pose
    }

    fun fetchNeutralSample1 (slide: LinearSlide, claw: AltClaw, follower: Follower, startPose: Pose): Pose {
        val p = PathBuilder()
        .addPath( // Line 6 - fetch the last neutral sample from the floor
            BezierLine(
//                Point(16.259, 128.206, Point.CARTESIAN),
                Point(startPose),
                Point(37.163, 133.315, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(325.0), Math.toRadians(225.0))
        runPath(p.build())
        return follower.pose
    }

    fun parkAndAscend (slide: LinearSlide, claw: AltClaw, follower: Follower, startPose: Pose): Pose {
        val p = PathBuilder()
        .addPath( // Line 8 - avoid the legs of the submersible
            BezierLine(
                Point(startPose),
                Point(59.924, 106.838, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(startPose.heading), Math.toRadians(0.0))
        .addPath( // Line 9 - park touching the low rung of the submersible in the ascension zone.
            BezierLine(
//                Point(59.924, 106.838, Point.CARTESIAN),
                Point(startPose),
                Point(59.924, 93.831, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(90.0))
        runPath(p.build())
        slide.setPosition(130.0, 260.0)

        return follower.pose
    }

}
