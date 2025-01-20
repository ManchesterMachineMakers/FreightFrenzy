package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subassemblies.AltClaw;
import org.firstinspires.ftc.teamcode.subassemblies.Follower;
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide;
import org.firstinspires.ftc.teamcode.util.ReleaseServo;

@Autonomous(name = "Basket Delivery", group = "OTOS", preselectTeleOp = "Alt Claw TeleOp")
public class BasketDelivery extends LinearOpMode {

    SparkFunOTOS.Pose2D STARTING_POSITION = new SparkFunOTOS.Pose2D(-61.8, 36, 0);
    static final double HIGH_BASKET_POS = 42;
    static final int ASCEND_POS = 20;

    private Follower follower;
    private LinearSlide linearSlide;
    private DcMotor linearSlideMotor;
    private Servo pinionServo;
    private AltClaw claw;
    private Servo wristServo;
    private ReleaseServo clawServo;

    @Override
    public void runOpMode() {
        follower = new Follower(this, STARTING_POSITION);

        linearSlide = new LinearSlide(this);
        linearSlideMotor = linearSlide.getLinearSlide();
        pinionServo = linearSlide.getPinion();

        claw = new AltClaw(this);
        wristServo = claw.getRotateServo();
        clawServo = claw.getClawServo();

        waitForStart();
        if (opModeIsActive()) {
            follower.driveToPos(
                    -47,
                    61,
                    -135,
                    2,
                    true
            );
            scoreSpecimen();
            follower.driveToPos(
                    -48,
                    36,
                    -135,
                    10,
                    false
            );
            follower.driveToPos(
                    -12,
                    36,
                    0,
                    10,
                    false
            );
            follower.driveToPos(
                    -12,
                    21.2,
                    0,
                    2,
                    true
            );
        }
    }

    private void scoreSpecimen() {
        wristServo.setPosition(0.8);
        linearSlide.getLinearSlide().setTargetPosition((int) (linearSlideMotor.getMotorType().getTicksPerRev() * (HIGH_BASKET_POS / (1.54 * Math.PI))));
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(1);
        while (linearSlideMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("linear slide pos", linearSlideMotor.getCurrentPosition());
        }
        linearSlideMotor.setPower(1);
        wristServo.setPosition(0.4);
        sleep(500);
        clawServo.open();
        sleep(500);
        wristServo.setPosition(0.8);
        sleep(500);
        linearSlideMotor.setTargetPosition(ASCEND_POS);
    }
}
