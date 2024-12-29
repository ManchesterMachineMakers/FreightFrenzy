package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Vision

@Autonomous(name = "Empty Autonomous")
class EmptyAuto: LinearOpMode() {

    override fun runOpMode() {
        waitForStart()

        val vision = Vision(this)

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                idle()
            }
        }
    }
}