package org.firstinspires.ftc.teamcode.opmodes.auto;

import static java.lang.Math.PI;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "AprilTagAuto")
public class AprilTagAuto extends LinearOpMode {

    private DcMotor left_rear;
    private DcMotor left_front;
    private DcMotor right_rear;
    private DcMotor right_front;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    boolean USE_WEBCAM;

    /**
     * This OpMode illustrates using a camera to locate and drive towards a specific
     * AprilTag. This OpMode assumes a Holonomic (Mecanum or X Drive) Robot.
     *
     * For an introduction to AprilTags, see the ftc-docs https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
     *
     * When an AprilTag in the TagLibrary is detected, the SDK provides location
     * and orientation of the tag, relative to the camera. This information is
     * provided in the "ftcPose" property of the returned "detection", and is explained
     * in the ftc-docs https://ftc-docs.firstinspires.org/apriltag-detection-values
     *
     * The driving goal is to rotate to keep the tag centered in the camera, while
     * strafing to be directly in front of the tag, and driving towards the tag to achieve
     * the desired distance. To reduce any motion blur (which will interrupt the detection
     * process) the Camera exposure is reduced to a very low value (5mS). You can determine
     * the best exposure and gain values by using the ConceptAprilTagOptimizeExposure OpMode.
     *
     * This OpMode assumes a Robot Configuration with motors named: leftfront_drive
     * and rightfront_drive, leftback_drive and rightback_drive. The motor directions
     * must be set so a positive power goes forward on all wheels. This sample
     * assumes that the default AprilTag Library (usually for the current season)
     * is being loaded by default, so you should choose to approach a valid tag ID.
     *
     * Under manual control, the left stick will move forward/back and
     * left/right. The right stick will rotate the robot. Manually drive
     * the robot until it displays Target data on the Driver Station.
     *
     * Press and hold the Left Bumper to enable the automatic "Drive to
     * target" mode. Release the Left Bumper to return to manual driving mode.
     *
     * Under "Drive To Target" mode, the robot has three goals:
     * 1) Turn the robot to always keep the Tag centered on the
     * camera frame. (Use the Target Bearing to turn the robot.)
     * 2) Strafe the robot towards the centerline of the Tag, so it approaches
     * directly in front of the tag. (Use the Target Yaw to strafe the robot.)
     * 3) Drive towards the Tag to get to the desired distance. (Use Tag Range to drive the robot forward/backward.)
     *
     * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
     * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN, and TURN_GAIN constants.
     */
    @Override
    public void runOpMode() {
        int DESIRED_DISTANCE;
        double SPEED_GAIN;
        double STRAFE_GAIN;
        double TURN_GAIN;
        double MAX_AUTO_SPEED;
        double MAX_AUTO_STRAFE;
        double MAX_AUTO_TURN;
        int DESIRED_TAG_ID;
        boolean targetFound;
        double drive;
        double strafe;
        double turn;
        AprilTagDetection desiredTag;
        List<AprilTagDetection> currentDetections;
        AprilTagDetection detection;
        double rangeError;
        double headingError;
        double yawError;

        left_rear = hardwareMap.get(DcMotor.class, "left_rearAsDcMotor");
        left_front = hardwareMap.get(DcMotor.class, "left_frontAsDcMotor");
        right_rear = hardwareMap.get(DcMotor.class, "right_rearAsDcMotor");
        right_front = hardwareMap.get(DcMotor.class, "right_frontAsDcMotor");

        // Adjust these numbers to suit your robot.
        // DESIRED_DISTANCE is how close the camera should get to the target in inches.
        DESIRED_DISTANCE = 26;
        // Set the GAIN constants to control the relationship between the measured position error, and
        // how much power is applied to the drive motors to correct the error.
        // Drive = Error * Gain
        // Make these values smaller for smoother control, or larger for a more aggressive response.
        // Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error. (0.50 / 25.0)
        SPEED_GAIN = 0.05;
        // Strafe Speed Control "Gain". e.g. Ramp up to 37% power at a 25 degree Yaw error. (0.375 / 25.0)
        STRAFE_GAIN = 0.04;
        // Turn Control "Gain". e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        TURN_GAIN = 0.01;
        // Clip the approach speed to this max value (adjust for your robot)
        MAX_AUTO_SPEED = 0.5;
        // Clip the strafing speed to this max value (adjust for your robot)
        MAX_AUTO_STRAFE = 0.5;
        // Clip the turn speed to this max value (adjust for your robot)
        MAX_AUTO_TURN = 0.3;
        // Set true to use a webcam, or false for a phone camera.
        USE_WEBCAM = true;
        // Choose the tag you want to approach or set to -1 for ANY tag.
        DESIRED_TAG_ID = -1;
        // Whether an AprilTag target is detected
        targetFound = false;
        // forward power/speed (-1 to +1)
        drive = 0;
        // strafe power/speed (-1 to +1)
        strafe = 0;
        // turning power/speed (-1 to +1)
        turn = 0;
        // Initialize the Apriltag Detection process.
        initAprilTag();
        // To drive forward, most robots need the motor on one side to be reversed, because the
        // axles point in opposite directions. When run, this OpMode should start both motors
        // driving forward. So adjust the motor directions based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.
        // Gear Reduction or 90 degree drives may require direction flips.
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_rear.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        if (USE_WEBCAM) {
            // Use low exposure time to reduce motion blur.
            setManualExposure(6, 250);
        }
        // Wait for the driver to press Start.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;
            // Step through the list of detected tags and look for a matching tag.
            currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection_item : currentDetections) {
                detection = detection_item;
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    // Check to see if we want to track towards this tag.
                    if (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        // Don't look any further.
                        break;
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID " + detection.id + " is not desired");
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID " + detection.id + " is not in TagLibrary");
                }
            }
            telemetry.addLine("");
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target");
                telemetry.addLine("");
                telemetry.addData("Found", "ID " + desiredTag.id + " (" + desiredTag.metadata.name + ")");
                telemetry.addData("Range", JavaUtil.formatNumber(desiredTag.ftcPose.range, 5, 1) + " inches");
                telemetry.addData("Bearing", JavaUtil.formatNumber(desiredTag.ftcPose.bearing, 3, 0) + " degrees");
                telemetry.addData("Yaw", JavaUtil.formatNumber(desiredTag.ftcPose.yaw, 3, 0) + " degrees");
            } else {
                telemetry.addData(">", "Drive using joysticks to find valid target");
                telemetry.addLine("");
            }
            // If Left Bumper is being pressed, and we have found the desired target, drive to target automatically.
            if (gamepad1.left_bumper && targetFound) {
                // Determine heading, range, and yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                headingError = desiredTag.ftcPose.bearing;
                yawError = desiredTag.ftcPose.yaw;
                // Use the speed and turn "gains" to calculate how we want the robot to move. Clip it to the maximum.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("Auto", "Drive " + JavaUtil.formatNumber(drive, 5, 2) + ", Strafe " + JavaUtil.formatNumber(strafe, 5, 2) + ", Turn " + JavaUtil.formatNumber(turn, 5, 2));
            } else {
                // Drive using manual POV Joystick mode. Slow things down to make the robot more controllable.
                // Reduce drive rate to 50%.
                drive = -gamepad1.left_stick_y / 2;
                // Reduce strafe rate to 50%.
                strafe = -gamepad1.left_stick_x / 2;
                // Reduce turn rate to 33%.
                turn = -gamepad1.right_stick_x / 3;
                telemetry.addData("Manual", "Drive " + JavaUtil.formatNumber(drive, 5, 2) + ", Strafe " + JavaUtil.formatNumber(strafe, 5, 2) + ", Turn " + JavaUtil.formatNumber(turn, 5, 2));
            }
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions.
     * Positive x is forward.
     * Positive y is strafe left.
     * Positive yaw is counter-clockwise.
     */
    private void moveRobot(double x, double y, double yaw) {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double max;

        // Calculate wheel powers.
        leftFrontPower = (x - y) - yaw;
        rightFrontPower = x + y + yaw;
        leftBackPower = (x - y) + yaw;
        rightBackPower = (x + y) - yaw;
        // Normalize wheel powers to be less than 1.0.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }
        // Send powers to the wheels.
        left_front.setPower(leftFrontPower);
        right_front.setPower(rightFrontPower);
        left_rear.setPower(leftBackPower);
        right_rear.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        AprilTagProcessor.Builder aprilTagBuilder;
        VisionPortal.Builder visionPortalBuilder;
        Position cameraPosition = new Position(DistanceUnit.MM, 168, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, -PI/2, 0, 0);

        // Create the AprilTag processor by using a builder.
        aprilTagBuilder = new AprilTagProcessor.Builder()
        // Set to Metric and Radians because we like them better!
            .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setCameraPose(cameraPosition, cameraOrientation);

        aprilTag = aprilTagBuilder.build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//            visionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);

        visionPortalBuilder.addProcessor(aprilTag);
        visionPortal = visionPortalBuilder.build();
    }

    /**
     * Manually set the camera gain and exposure.
     * This can only be called after calling initAprilTag, and only works for webcams.
     */
    private void setManualExposure(int exposureMs, int gain) {
        ExposureControl exposureControl;
        GainControl gainControl;

        // Wait for the camera to be open, then use the controls.
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls.
        if (!visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && !visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (!exposureControl.getMode().equals(ExposureControl.Mode.Manual)) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            sleep(20);
            gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}