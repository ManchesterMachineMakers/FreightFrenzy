package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision

@Autonomous(name = "Sample Autonomous")
class SampleAuto: LinearOpMode() {
  
    override fun runOpMode() {

        val driveBase = MecDriveBase(this)
        val vision = Vision(this)
        val follower = Follower(driveBase)

        vision.aprilTag
        follower.initialize()

        waitForStart()

        if (opModeIsActive()) {
            
        }
    }
}
