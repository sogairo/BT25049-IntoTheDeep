package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enlightenment.MecanumDrive;

@Autonomous(name = "BT25046-Auton", group = "Competition")
public class Eureka extends LinearOpMode {
    // Initialize Constants
    // ------------------------------------------------ //
    public static class Configuration {
        public double DRIVE_POWER_CONSTRAINT = 0.8;
        public double SPOOL_ASCENT_CONSTRAINT = 0.6;

        public int MAX_SPOOL_POSITION = 1610;
        public int MIN_SPOOL_POSITION = 0;
        public int SPOOL_INCREMENT = 30;
        public int SPOOL_POSITION_TOLERANCE = 15;
        public int SAMPLE_BASKET_POSITION = 1610;
        public int SPECIMEN_POSITION = 415;

        public double MAX_ARM_POSITION = 0.985;
        public double MIN_ARM_POSITION = 0;
        public double ARM_SAMPLE_POSITION = 0.5;
        public double ARM_SPECIMEN_POSITION = 0.48;
        public double ARM_GRAB_POSITION = 0.92;
        public double ARM_INCREMENT = 0.04;

        public double MIN_CLAW_POSITION = 0.455;
        public double MAX_CLAW_POSITION = 0.625;
    }

    public static Eureka.Configuration Params = new Eureka.Configuration();

    // Main Constants
    // ------------------------------------------------ //
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Claw clawServo = new Claw(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .waitSeconds(2)
                        .stopAndAdd(clawServo.activateClaw())
                        .build());
    }

    public static class Claw {
        private final Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "claw");
            clawServo.scaleRange(Params.MIN_CLAW_POSITION, Params.MAX_CLAW_POSITION);
            clawServo.setPosition(1);
        }

        public class ClawFunction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (clawServo.getPosition() == 0) {
                    clawServo.setPosition(1);
                } else {
                    clawServo.setPosition(0);
                }
                return false;
            }
        }

        public Action activateClaw() {
            return new ClawFunction();
        }
    }
}
