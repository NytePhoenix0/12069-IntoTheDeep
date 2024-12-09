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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        boolean safe_mode = true;
        double speedFactor = 1.0;
        boolean yIsPressed = false;
        boolean aIsPressed = false;
        boolean override = false;

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

            if(gamepad2.y && !yIsPressed) {
                yIsPressed = true;
                override = !override;
            }
            if(!gamepad2.y) {
                yIsPressed = false;
            }

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

            if(gamepad2.a && !aIsPressed) {
                aIsPressed = true;
                if(slideServo.getPosition() == 0) {
                    slideServo.setPosition(0.9);
                } else {
                    slideServo.setPosition(0);
                }
            }
            if(!gamepad2.a) {
                aIsPressed = false;
            }

            pivotArmMotor.setPower(gamepad2.left_stick_y * 0.2);

            if(!(override && (-extArmMotor.getCurrentPosition() < 0 && -armPow < 0 || -extArmMotor.getCurrentPosition() > 850 && -armPow > 0))) {
                extArmMotor.setPower(gamepad2.right_stick_y * 0.4);
            } else if(override) {
                extArmMotor.setPower(gamepad2.right_stick_y * 0.4);
            } else {
                extArmMotor.setPower(0);
            }

            frontLeftMotor.setPower(-frontLeftPower * speedFactor);
            backLeftMotor.setPower(-backLeftPower * speedFactor);
            frontRightMotor.setPower(-frontRightPower * speedFactor);
            backRightMotor.setPower(-backRightPower * speedFactor);

            telemetry.addData("Arm Position", -extArmMotor.getCurrentPosition());
            telemetry.addData("Arm Power", -armPow);
            telemetry.addData("override", override);
            telemetry.addData("Slide Servo position", slideServo.getPosition());
            telemetry.update();
        }
    }
}