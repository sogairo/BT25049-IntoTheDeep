package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.enlightenment.MecanumDrive;

@Autonomous(name = "BT25046-SampleAutonomous", group = "Competition")
public class Elucidate extends LinearOpMode {
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
            armServo.setDirection(Servo.Direction.REVERSE);
            armServo.setPosition(Params.MIN_ARM_POSITION);
        }

        public class DropSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.ARM_SAMPLE_POSITION);
                return false;
            }
        }

        public Action dropSample() {
            return new DropSample();
        }

        public class GrabSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.ARM_GRAB_POSITION);
                return false;
            }
        }

        public Action grabSample() {
            return new GrabSample();
        }

        public class LowerArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.MAX_ARM_POSITION);
                return false;
            }
        }

        public Action lowerArm() {
            return new LowerArm();
        }

        public class ResetArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.MIN_ARM_POSITION);
                return false;
            }
        }

        public Action resetArm() {
            return new ResetArm();
        }
    }

    public static class LinearSlides {
        private final VoltageSensor voltageSensor;
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

            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        public class RaiseSlides implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentSpoolPosition = Params.SAMPLE_BASKET_POSITION;
                    initialized = true;
                }

                controller.setPIDF(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

                spoolLeft.setTargetPosition(currentSpoolPosition);
                spoolRight.setTargetPosition(currentSpoolPosition);

                double voltage = voltageSensor.getVoltage();
                double averagePID = ((controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition)
                        + controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition)) / 2) / voltage;

                spoolLeft.setPower(averagePID);
                spoolRight.setPower(averagePID);

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

                double voltage = voltageSensor.getVoltage();
                double averagePID = ((controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition)
                        + controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition)) / 2) / voltage;

                spoolLeft.setPower(averagePID);
                spoolRight.setPower(averagePID);

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
        Pose2d initalStartingPose = new Pose2d(-38, -61.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initalStartingPose);

        Claw claw = new Claw(hardwareMap);
        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Action dropPreloadSample = drive.actionBuilder(initalStartingPose)
                .strafeToLinearHeading(new Vector2d(-59, -55), Math.toRadians(225))
                .stopAndAdd(linearSlides.raiseSlides())
                .stopAndAdd(arm.dropSample())
                .waitSeconds(1.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .build();

        Action getSampleOne = drive.actionBuilder(new Pose2d(-59, -55, 0))
                .strafeToLinearHeading(new Vector2d(-46.5, -47), Math.toRadians(80))
                .stopAndAdd(linearSlides.lowerSlides())
                .stopAndAdd(arm.lowerArm())
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(1)
                .stopAndAdd(arm.resetArm())
                .strafeToLinearHeading(new Vector2d(-59, -53), Math.toRadians(225))
                .stopAndAdd(linearSlides.raiseSlides())
                .stopAndAdd(arm.dropSample())
                .waitSeconds(1.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .build();

        Action getSampleTwo = drive.actionBuilder(new Pose2d(-59, -55, 0))
                .strafeToLinearHeading(new Vector2d(-59, -47), Math.toRadians(80))
                .stopAndAdd(linearSlides.lowerSlides())
                .stopAndAdd(arm.lowerArm())
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(1)
                .stopAndAdd(arm.resetArm())
                .strafeToLinearHeading(new Vector2d(-59, -53), Math.toRadians(225))
                .stopAndAdd(linearSlides.raiseSlides())
                .stopAndAdd(arm.dropSample())
                .waitSeconds(1.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                        dropPreloadSample,
                        getSampleOne,
                        getSampleTwo
                )
        );
    }
}
