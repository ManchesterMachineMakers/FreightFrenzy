package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Subassembly(protected val opMode: OpMode, protected val name: String) {

    var status = "unknown"
    protected val telemetry: Telemetry
        get() { // reroute telemetry to dashboard if supported
            return if (this.javaClass.interfaces.contains(DashOpMode::class.java)) MultipleTelemetry(DashOpMode.Static.telemetry, opMode.telemetry)
            else opMode.telemetry
        }
    protected val hardwareMap = opMode.hardwareMap
    protected val runtime = opMode.runtime

    init {
        hardwareMap ?: throw NullPointerException("${opMode::class}.hardwareMap is NULL. Make sure your subassembly is initialized in runOpMode()")
    }

    open fun telemetry() {
        telemetry.addLine(name)
    }
}