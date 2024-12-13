package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry.Line
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit

class Vision(private val opMode: OpMode) {
    private val hardwareMap = opMode.hardwareMap
    private val webcam = hardwareMap.get(WebcamName::class.java, "Webcam 1")

    private val telemetry = opMode.telemetry

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

    /**
     * Returns the distance away from the specified AprilTag.
     * @param desiredTagId The ID of the tag.
     */
    fun aprilTagDistance(desiredTagId: Int): Double? {
        val desiredTag = aprilTag.detections
            .filter { it.metadata != null }
            .find { it.metadata.id == desiredTagId || desiredTagId < 0 }

        if(desiredTag == null) {
            telemetry.addLine("AprilTag $desiredTagId not found.")
            telemetry.update()
            return null
        }

        return desiredTag.ftcPose.range
    }


    private fun setManualExposure(exposureMS: Int, gain: Int) {
        if(!(opMode is LinearOpMode)) return

        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting")
            telemetry.update()
            while (!opMode.isStopRequested && (visionPortal.cameraState != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20)
            }
            telemetry.addData("Camera", "Ready")
            telemetry.update()
        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested) {
            val exposureControl = visionPortal.getCameraControl(
                ExposureControl::class.java
            )
            if (exposureControl.mode != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual)
                opMode.sleep(50)
            }
            exposureControl.setExposure(exposureMS.toLong(), TimeUnit.MILLISECONDS)
            opMode.sleep(20)
            val gainControl = visionPortal.getCameraControl(
                GainControl::class.java
            )
            gainControl.setGain(gain)
            opMode.sleep(20)
        }
    }

    init {
        while(visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {}
        ptzControl = visionPortal.getCameraControl(PtzControl::class.java)

        setManualExposure(6, 250) // Use low exposure time to reduce motion blur


        opMode.log("Vision successfully initialized")
    }
}