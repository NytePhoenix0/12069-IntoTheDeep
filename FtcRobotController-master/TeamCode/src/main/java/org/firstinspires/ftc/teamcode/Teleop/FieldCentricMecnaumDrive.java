package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field_Centric_Mecnaum")
public class FieldCentricMecnaumDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor =  hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
        waitForStart();

        if (isStopRequested()) return;

        double speedFactor = 1.0;
        while (opModeIsActive())  {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double up = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

           if (gamepad1.a) {
               imu = hardwareMap.get(IMU.class, "imu");
               parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                       RevHubOrientationOnRobot.LogoFacingDirection.UP,
                       RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
               imu.initialize(parameters);
               imu.resetYaw();
               botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            if (gamepad1.dpad_left && speedFactor > 0.1){
                speedFactor -= 0.1;
            } if (gamepad1.dpad_right && speedFactor < 1.0) {
                speedFactor += 0.1;
            }

            double rotX = x * Math.cos(-botHeading) - up * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + up * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator);
            double backLeftPower = ((rotY - rotX + rx) / denominator);
            double frontRightPower = ((rotY - rotX - rx) / denominator);
            double backRightPower = ((rotY + rotX - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower * speedFactor);
            backLeftMotor.setPower(backLeftPower * speedFactor);
            frontRightMotor.setPower(frontRightPower * speedFactor);
            backRightMotor.setPower(backRightPower * speedFactor);
            
            telemetry.update();

        }
    }
}
