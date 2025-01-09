package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BT25046-Autonomous", group = "Competition")
public class RoadRunnerAutonomous extends LinearOpMode {
    final Pose2d StartPosition = new Pose2d(-24, -60, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive DriveTrain = new SampleMecanumDrive(hardwareMap);
        DriveTrain.setPoseEstimate(StartPosition);

        TrajectorySequence MainTrajectory = DriveTrain.trajectorySequenceBuilder(StartPosition)
                .build();

        waitForStart();
        if (opModeIsActive() && !isStopRequested()) {
            DriveTrain.followTrajectorySequence(MainTrajectory);
        }
    }
}
