package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase

class Vision(opMode: LinearOpMode) {
    private val hardwareMap = opMode.hardwareMap
    private val webcam = hardwareMap.get(WebcamName::class.java, "Webcam 1")

    val dash = DashOpMode.CameraStreamProcessor()

    // http://localhost:63342/RobotController/Vision-9.0.1-javadoc.jar/org/firstinspires/ftc/vision/apriltag/AprilTagProcessor.Builder.html
    val aprilTag = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .build()

    val visionPortal = VisionPortal.Builder()
            .setCamera(webcam)
            .addProcessors(aprilTag, dash)
            .build()

    var ptzControl: PtzControl? = null // let OpMode control
        private set

    init {
        while(visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {}
        ptzControl = visionPortal.getCameraControl(PtzControl::class.java)

        opMode.log("Vision successfully initialized")
    }
}