package org.firstinspires.ftc.teamcode.opmodes.auto

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
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.toRadians

@Config
@Autonomous(name = "Blue DoNotBreakThisPark")
class BlueDoNotBreakThisPark: LinearOpMode(), DashOpMode {

    override fun runOpMode() {

        val driveBase = MecDriveBase(this)
        val follower = Follower(driveBase, hardwareMap)
        val telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)

        val pathBuilder = PathBuilder() // path generated with https://pedro-path-generator.vercel.app/
            .addPath(
                BezierCurve(
                    Point(startingPosX, startingPosY, Point.CARTESIAN),
                    Point(-60.0, 56.0, Point.CARTESIAN),
                    Point(-60.0, 60.0, Point.CARTESIAN)
                )

            ).setConstantHeadingInterpolation(startingHeading.toRadians());

        val path = pathBuilder.build()

        follower.initialize()
        follower.setStartingPose(Pose(startingPosX, startingPosY, startingHeading.toRadians()))

        waitForStart()

        if (opModeIsActive()) {

            follower.followPath(path, true)

            while (opModeIsActive()) {

                follower.update()
                follower.telemetryDebug(telemetryA)
            }
        }
    }

    @JvmField var startingPosX = -12.0
    @JvmField var startingPosY = 60.0
    @JvmField var startingHeading = -180.0 // degrees

}
