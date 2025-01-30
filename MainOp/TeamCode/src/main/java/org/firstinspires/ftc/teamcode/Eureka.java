package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enlightenment.MecanumDrive;

@Autonomous(name = "BT25046-SpecimenAutonomous", group = "Competition")
public class Eureka extends LinearOpMode {
    // Initialize Constants
    // ------------------------------------------------ //
    public static Epiphany.Configuration Params = new Epiphany.Configuration();

    // Initialize PIDF
    // ------------------------------------------------ //
    public static Epiphany.PIDFConfiguration PIDFParams = new Epiphany.PIDFConfiguration();
    private static final PIDFController controller = new PIDFController(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

    // Initalize Variables
    // ------------------------------------------------ //
    public static int currentSpoolPosition = Params.MIN_SPOOL_POSITION;
    private static final int setPositionType = Params.SPECIMEN_POSITION;

    // Create Actions for Objects
    // ------------------------------------------------ //
    public static class Claw {
        private final Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "claw");
            clawServo.scaleRange(Params.MIN_CLAW_POSITION, Params.MAX_CLAW_POSITION);
            clawServo.setPosition(0);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(1);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public static class Arm {
        private final Servo armServo;

        public Arm(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class, "armLeft");
            armServo.setPosition(Params.MIN_ARM_POSITION);
        }

        public class RaiseArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.ARM_SPECIMEN_POSITION);
                return false;
            }
        }

        public Action raiseArm() {
            return new RaiseArm();
        }

        public class LowerArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.MIN_ARM_POSITION);
                return false;
            }
        }

        public Action lowerArm() {
            return new LowerArm();
        }
    }

    public static class LinearSlides {
        private final DcMotorEx spoolLeft, spoolRight;

        public LinearSlides(HardwareMap hardwareMap) {
            spoolLeft = hardwareMap.get(DcMotorEx.class, "spoolLeft"); // Port 0
            spoolRight = hardwareMap.get(DcMotorEx.class, "spoolRight"); // Port 1

            spoolRight.setDirection(DcMotorSimple.Direction.REVERSE);

            spoolLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spoolRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            spoolLeft.setTargetPosition(Params.MIN_SPOOL_POSITION);
            spoolRight.setTargetPosition(Params.MIN_SPOOL_POSITION);
            spoolLeft.setTargetPositionTolerance(Params.SPOOL_POSITION_TOLERANCE);
            spoolRight.setTargetPositionTolerance(Params.SPOOL_POSITION_TOLERANCE);

            spoolLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spoolRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            spoolLeft.setPower(0.01);
            spoolRight.setPower(0.01);
        }

        public class RaiseSlides implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentSpoolPosition = Params.SPECIMEN_POSITION;
                    initialized = true;
                }

                controller.setPIDF(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

                spoolLeft.setTargetPosition(currentSpoolPosition);
                spoolRight.setTargetPosition(currentSpoolPosition);

                spoolLeft.setPower(Math.min(controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));
                spoolRight.setPower(Math.min(controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));

                return ((spoolLeft.getCurrentPosition() >= (currentSpoolPosition - Params.SPOOL_POSITION_TOLERANCE)) &&
                        (spoolRight.getCurrentPosition() >= (currentSpoolPosition - Params.SPOOL_POSITION_TOLERANCE)));
            }
        }

        public Action raiseSlides() {
            return new RaiseSlides();
        }

        public class LowerSlides implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentSpoolPosition = Params.MIN_SPOOL_POSITION;
                    initialized = true;
                }

                controller.setPIDF(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

                spoolLeft.setTargetPosition(currentSpoolPosition);
                spoolRight.setTargetPosition(currentSpoolPosition);

                spoolLeft.setPower(Math.min(controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));
                spoolRight.setPower(Math.min(controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));

                return ((spoolLeft.getCurrentPosition() <= (currentSpoolPosition + Params.SPOOL_POSITION_TOLERANCE)) &&
                        (spoolRight.getCurrentPosition() <= (currentSpoolPosition + Params.SPOOL_POSITION_TOLERANCE)));
            }
        }

        public Action lowerSlides() {
            return new LowerSlides();
        }
    }

    // Main OpMode
    // ------------------------------------------------ //
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initalStartingPose = new Pose2d(0, 61.5,  Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initalStartingPose);

        Claw claw = new Claw(hardwareMap);
        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        // Develop Paths
        Action hangPreloadSpecimen = drive.actionBuilder(initalStartingPose)
                .lineToY(25)
                .build();

        Action grabFirstSample = drive.actionBuilder(new Pose2d(0, 25,  Math.toRadians(-90)))
                //.setTangent(Math.toRadians(180))
                .strafeTo(new Vector2d(-48.1, 39))
                .strafeTo(new Vector2d(-58.5, 39))
                //.splineTo(new Vector2d(-48, 14), Math.toRadians(-180))
                .build();

        Action simultaneousHang = drive.actionBuilder(initalStartingPose)
                .waitSeconds(0.4)
                .stopAndAdd(claw.openClaw())
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                        // raise arm + slides while going to specimen bar
                        new ParallelAction(
                                arm.raiseArm(),
                                linearSlides.raiseSlides()
                        ),
                        hangPreloadSpecimen,

                        // hang
                        new ParallelAction(
                                linearSlides.lowerSlides()
                        ),
                        simultaneousHang,

                        new ParallelAction(
                                arm.lowerArm()
                        ),
                        // go to first sample
                        grabFirstSample
                )
        );
    }
}
