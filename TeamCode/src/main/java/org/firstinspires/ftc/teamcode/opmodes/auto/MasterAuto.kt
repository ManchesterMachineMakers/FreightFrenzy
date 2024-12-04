package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision

class MasterAuto(val alliance: Alliance, val startPosition: StartPosition, val goal: Goal): LinearOpMode() {

    lateinit var driveBase: MecDriveBase
    lateinit var vision: Vision

    enum class Alliance {
        RED, BLUE
    }

    enum class StartPosition {
        FRONT, BACK
    }

    enum class Goal {
        PARK, LVL1_ASCEND, SCORE_SPECIMEN
    }

    override fun runOpMode() {

        driveBase = MecDriveBase(this)
        vision = Vision(this)

    }

    fun getStartPose(): Pose {
        val y = if (startPosition == StartPosition.FRONT) frontStartY
                else backStartY
        val x = if (alliance == Alliance.BLUE) wallOffset - 72.0
                else 72.0 - wallOffset
        return Pose(x, y)
    }

    fun getPath(): Path {
        return when (goal) {
            Goal.PARK -> Path(
                BezierCurve(
                    Point(getStartPose()),
                )
            )
            else -> Path(
                BezierCurve()
            )
        }
    }

    @Config
    companion object {
        @JvmField var frontStartY = -12.0 // in
        @JvmField var backStartY = 36.0 // in
        @JvmField var wallOffset = 12.0 // distance from center of robot to wall
    }
}