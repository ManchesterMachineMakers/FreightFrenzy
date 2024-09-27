package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.log

class Vision(opMode: LinearOpMode) {
    private val hardwareMap = opMode.hardwareMap
    private val webcam = hardwareMap.get(WebcamName::class.java, "Webcam 1")

    // http://localhost:63342/RobotController/Vision-9.0.1-javadoc.jar/org/firstinspires/ftc/vision/apriltag/AprilTagProcessor.Builder.html
    val aprilTag = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
            .build()

    val visionPortal = VisionPortal.Builder()
            .setCamera(webcam)
            .addProcessors(aprilTag)
            .build()

    init {
        while(visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {}
        val ptzControl = visionPortal.getCameraControl(PtzControl::class.java)
        ptzControl.setZoom(150)

        opMode.log("Vision successfully initialized")
    }
}