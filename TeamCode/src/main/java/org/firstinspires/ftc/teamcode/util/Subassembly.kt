package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class Subassembly(protected val opMode: OpMode, protected val name: String) {

    var status = "unknown"
    protected val telemetry = opMode.telemetry
    protected val hardwareMap = opMode.hardwareMap
    protected val runtime = opMode.runtime

    init {
        hardwareMap ?: throw NullPointerException("${opMode::class}.hardwareMap is NULL. Make sure your subassembly is initialized in runOpMode()")
    }

    open fun telemetry() {
        telemetry.addLine(name)
    }
}