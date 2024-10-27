package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.util.log

@TeleOp(name = "Main TeleOp (preferred)", group = "")
class MainTeleOp: LinearOpMode() {

    override fun runOpMode() {
        // init, no movement allowed
        telemetry.isAutoClear = false

        val driveBase = DriveBase(this)
        // add other subassemblies here

        val loopTime = ElapsedTime()
        driveBase.opInit()

        waitForStart()

        if (opModeIsActive()) {
            log("Starting OpMode loop")
            telemetry.isAutoClear = true
            telemetry.clear()
            while (opModeIsActive()) {
                loopTime.reset()
                telemetry.addData("G1 Left X", gamepad1.left_stick_x)
                telemetry.addData("G1 Left Y", gamepad1.left_stick_y)
                telemetry.addData("G1 Right X", gamepad1.right_stick_x)
                telemetry.addData("G1 Right XY", gamepad1.right_stick_y)

                // Subassembly control
                driveBase.control(gamepad1)
                // control other subassemblies here

                telemetry.addData("Loop Time", loopTime.time())
                telemetry.update()
            }
        }
    }

//    fun telemetry() {
//        telemetry.addData("runtime (s)", runtime)
//        telemetry.addData("looptime (ms)", loopTime.milliseconds())
//        runAllTelemetries()
//    }

//    fun runAllTelemetries() {
//        for (subassembly in subassemblyList) subassembly?.telemetry()
//        telemetry.update()
//    }
}