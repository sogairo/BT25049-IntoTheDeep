package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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

    // Main OpMode
    // ------------------------------------------------ //
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Claw clawServo = new Claw(hardwareMap);
        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(61.5, 0, 0))
                        .strafeTo(new Vector2d(33, 0))
                        .waitSeconds(1)
                        .stopAndAdd(arm.activateArm())
                        .stopAndAdd(linearSlides.activateSlides())
                        .waitSeconds(1)
                        .stopAndAdd(linearSlides.activateSlides())
                        .stopAndAdd(clawServo.activateClaw())
                        .waitSeconds(2)
                        .build());
                /*
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .stopAndAdd(arm.activateArm())
                                .stopAndAdd(linearSlides.activateSlides())
                                .build());


                 */
    }

    public static class Claw {
        private final Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "claw");
            clawServo.scaleRange(Params.MIN_CLAW_POSITION, Params.MAX_CLAW_POSITION);
            clawServo.setPosition(0);
        }

        // It works but you need to wait secords until it does work...
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

    public static class Arm {
        private final Servo armServo;

        public Arm(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class, "armLeft");
            armServo.setPosition(Params.MIN_ARM_POSITION);
        }

        // It works but you need to wait secords until it does work...
        public class ArmFunction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.ARM_SPECIMEN_POSITION);
                return false;
            }
        }

        public Action activateArm() {
            return new ArmFunction();
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

        // It works but you need to wait secords until it does work...
        public class LinearSlideFunction implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    if (currentSpoolPosition != setPositionType) {
                        currentSpoolPosition = setPositionType;
                    } else {
                        currentSpoolPosition = Params.MIN_SPOOL_POSITION;
                    }
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

        public Action activateSlides() {
            return new LinearSlideFunction();
        }
    }
}
