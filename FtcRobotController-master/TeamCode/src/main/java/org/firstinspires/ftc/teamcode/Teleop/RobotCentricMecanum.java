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

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

//        int prevPosition = 0;
        double speedFactor = 1.0;
        boolean override = false;
        boolean yIsPressed = false;
        boolean ltIsPressed = false;
        boolean rtIsPressed = false;

        while (opModeIsActive()) {
            double y = gamepad1.right_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x - rx) / denominator);
            double backLeftPower = ((y - x - rx) / denominator);
            double frontRightPower = ((y - x + rx) / denominator);
            double backRightPower = ((y + x + rx) / denominator);
            double resist = 0;

//            // prevent flopping (maybe?) - untested
//            if(pivotArmMotor.getCurrentPosition() - prevPosition > 10) {
//                resist = - 0.1;
//            } else if(pivotArmMotor.getCurrentPosition() - prevPosition < -10) {
//                resist = 0.1;
//            } else {
//                resist = 0;
//            }
//            prevPosition = pivotArmMotor.getCurrentPosition();

            // toggle override for extension arm limits
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

            // extension arm with limits
            if(override) {
                extArmMotor.setPower(gamepad2.left_stick_y * 0.7);
            } else if(!(-extArmMotor.getCurrentPosition() < 0 && -gamepad2.left_stick_y < 0 || -extArmMotor.getCurrentPosition() > 1300  && -gamepad2.left_stick_y > 0)) {
                extArmMotor.setPower(gamepad2.left_stick_y * 0.7);
            } else {
                extArmMotor.setPower(0);
            }

            // set bucket position to starting config
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
            pivotArmMotor.setPower(gamepad2.right_stick_y * 0.3 + resist);

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

            // drive
            frontLeftMotor.setPower(-frontLeftPower * speedFactor);
            backLeftMotor.setPower(-backLeftPower * speedFactor);
            frontRightMotor.setPower(-frontRightPower * speedFactor);
            backRightMotor.setPower(-backRightPower * speedFactor);

            // drive gears - untested
            if(gamepad1.dpad_down /* && speedFactor > 0.2 && !ltIsPressed */) {
//                ltIsPressed = true;
                speedFactor = 0.3;
            }
//            if(!(gamepad1.dpad_down)) {
//                ltIsPressed = false;
//            }
            if(gamepad1.dpad_up /* && speedFactor < 1 && ! */) {
//                rtIsPressed = true;
                speedFactor = 1;
            }
//            if(!(gamepad1.dpad_up)) {
//                rtIsPressed = false;
//            }

            telemetry.addData("extension arm position", -extArmMotor.getCurrentPosition());
            telemetry.addData("override", override);
            telemetry.addData("speed factor", speedFactor);
            telemetry.update();
        }
    }
}