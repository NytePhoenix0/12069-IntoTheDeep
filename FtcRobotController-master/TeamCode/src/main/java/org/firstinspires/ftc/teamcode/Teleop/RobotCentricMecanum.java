package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "RobotCentricMecanum")
public class RobotCentricMecanum extends LinearOpMode {
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        YawPitchRollAngles RobotOrientation;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        CRServo leftServo = hardwareMap.crservo.get("leftServo");
        CRServo rightServo = hardwareMap.crservo.get("rightServo");
        Servo slideServo = hardwareMap.servo.get("slideServo");
        DcMotor extArmMotor = hardwareMap.dcMotor.get("extensionArm");
        DcMotor pivotArmMotor = hardwareMap.dcMotor.get("pivotArm");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivotArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        double speedFactor = 1.0;
        boolean yIsPressed = false;
//        boolean lbIsPressed = false;
        boolean override = false;
        int prevPosition = 0;

        while (opModeIsActive()) {
            double y = gamepad1.right_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;
            double armPow = gamepad2.right_stick_y;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator);
            double backLeftPower = ((y - x + rx) / denominator);
            double frontRightPower = ((y - x - rx) / denominator);
            double backRightPower = ((y + x - rx) / denominator);
            double resist = 0;

            // prevent flopping (maybe)
            if(pivotArmMotor.getCurrentPosition() - prevPosition > 10) {
                resist = - 0.1;
            } else if(pivotArmMotor.getCurrentPosition() - prevPosition < -10) {
                resist = 0.1;
            } else {
                resist = 0;
            }
            prevPosition = pivotArmMotor.getCurrentPosition();

            // toggle override for arm limits
            // note: maybe reset encoder when turning off override or add a different button to reset the encoder
            if(gamepad2.y && !yIsPressed) {
                yIsPressed = true;
                override = !override;
            }
            if(!gamepad2.y) {
                yIsPressed = false;
            }

            // reset extension arm encoder
            if(gamepad2.right_bumper) {
                extArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // intake: up = out, down = in
            if(gamepad2.dpad_up) {
                leftServo.setPower(1);
                rightServo.setPower(-1);
            } else if(gamepad2.dpad_down) {
                leftServo.setPower(-1);
                rightServo.setPower(1);
            } else {
                leftServo.setPower(0);
                rightServo.setPower(0);
            }

//            // bucket (toggle)
//            if(gamepad1.left_bumper && !lbIsPressed) {
//                lbIsPressed = true;
//                if(slideServo.getPosition() == 0) {
//                    slideServo.setPosition(0.9);
//                } else {
//                    slideServo.setPosition(0);
//                }
//            }
//            if(!gamepad1.left_bumper) {
//                aIsPressed = false;
//            }

            if(gamepad1.x) {
                slideServo.setPosition(0.9);
            }
            // bucket: left = down, right = up
            if(gamepad1.left_bumper) {
                slideServo.setPosition(0.66);
            }
            if(gamepad1.right_bumper) {
                slideServo.setPosition(0);
            }

            // pivot arm
            // note: add constant power to prevent flopping from momentum
            pivotArmMotor.setPower(gamepad2.left_stick_y * 0.3 + resist);

            // extension arm with limits
            // note: add constant power to avoid slipping, adjust limits
            if(override) {
                extArmMotor.setPower(gamepad2.right_stick_y * 0.4);
            } else if(!(-extArmMotor.getCurrentPosition() < 0 && -armPow < 0 || -extArmMotor.getCurrentPosition() > 12  && -armPow > 0)) {
                extArmMotor.setPower(gamepad2.right_stick_y * 0.4);
            } else {
                extArmMotor.setPower(0);
            }

            // drive
            frontLeftMotor.setPower(-frontLeftPower * speedFactor);
            backLeftMotor.setPower(-backLeftPower * speedFactor);
            frontRightMotor.setPower(-frontRightPower * speedFactor);
            backRightMotor.setPower(-backRightPower * speedFactor);

            telemetry.addData("extension arm position", -extArmMotor.getCurrentPosition());
            telemetry.addData("extension arm power", -armPow);
            telemetry.addData("override", override);
            telemetry.addData("slide servo position", slideServo.getPosition());
            telemetry.addData("pivot arm position", pivotArmMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}