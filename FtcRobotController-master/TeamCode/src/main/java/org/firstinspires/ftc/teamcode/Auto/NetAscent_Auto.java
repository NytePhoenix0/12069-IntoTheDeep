package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "NetAscent_Auto")
public class NetAscent_Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor extArmMotor = hardwareMap.dcMotor.get("extensionArm");
        extArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(60)
                .strafeLeft(10)
                .back(58)
                .forward(58)
                .strafeLeft(8)
                .back(58)
                .forward(33)
                .turn(Math.toRadians(90))
                .forward(10)
                .strafeLeft(35)
                .back(20)
                .strafeRight(58)
                .turn(Math.toRadians(-180))
                .forward(20)
                .addTemporalMarker(19, () -> {
                    while(extArmMotor.getCurrentPosition() < 200) {
                        extArmMotor.setPower(0.2);
                    }
                    extArmMotor.setPower(0);
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
