package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BT25046-Autonomous", group = "Competition")
public class RoadRunnerAutonomous extends LinearOpMode {
    // Initialize Constants
    final Pose2d StartPosition = new Pose2d(-24, -60, Math.toRadians(90));

    // Initialize Variables.
    private DcMotor spoolLeftMotor;
    private DcMotor spoolRightMotor;

    private int currentSpoolPosition = 0;

    // Create new thread to run the PID tuning
    Thread PIDUpdater = new Thread(() -> {
        PIDController controller;
        double p = 0.004, i = 0.2, d = 0.02;
        controller = new PIDController(p, i ,d);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            controller.setPID(p, i, d);

            spoolLeftMotor.setPower(controller.calculate(spoolLeftMotor.getCurrentPosition(), currentSpoolPosition));
            spoolLeftMotor.setTargetPosition(currentSpoolPosition);

            spoolRightMotor.setPower(controller.calculate(spoolRightMotor.getCurrentPosition(), currentSpoolPosition));
            spoolRightMotor.setTargetPosition(currentSpoolPosition);
        }
    });

    @Override
    public void runOpMode() throws InterruptedException {
        // Claw Configuration.
        Servo remadeClawServo = hardwareMap.servo.get("remadeClaw");

        // Claw Pivot Configuration.
        Servo clawPivot = hardwareMap.servo.get("clawPivot");

        // Arm Configuration.
        Servo armRightServo = hardwareMap.servo.get("armRight");
        armRightServo.setDirection(Servo.Direction.REVERSE);

        Servo armLeftServo = hardwareMap.servo.get("armLeft");

        // Spool Linear Slides Configuration.
        spoolLeftMotor = hardwareMap.dcMotor.get("spoolLeft");
        spoolLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spoolLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spoolLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spoolLeftMotor.setTargetPosition(0);
        spoolLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spoolLeftMotor.setPower(1);

        spoolRightMotor = hardwareMap.dcMotor.get("spoolRight");
        spoolRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spoolRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spoolRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spoolRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spoolRightMotor.setTargetPosition(0);
        spoolRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spoolRightMotor.setPower(1);

        // RoadRunner Setup
        SampleMecanumDrive DriveTrain = new SampleMecanumDrive(hardwareMap);
        DriveTrain.setPoseEstimate(StartPosition);
        PIDUpdater.start(); // Start the PID Updater for the Arm

        // Build Paths
        TrajectorySequence MainTrajectory = DriveTrain.trajectorySequenceBuilder(StartPosition)
                // arm pid example
                .addTemporalMarker(() -> {
                    currentSpoolPosition = 600;
                })

                // servo example
                .addTemporalMarker(() -> {
                    remadeClawServo.setPosition(1);
                })
                .build();

        waitForStart();
        if (opModeIsActive() && !isStopRequested()) {
            DriveTrain.followTrajectorySequence(MainTrajectory);
        }
    }
}
