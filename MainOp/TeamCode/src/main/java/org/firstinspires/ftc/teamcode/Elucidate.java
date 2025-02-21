package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

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
    public static double currentArmPosition = Params.MIN_ARM_POSITION;

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

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0);
                return false;
            }
        }

        public Action close() {
            return new Close();
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(1);
                return false;
            }
        }

        public Action open() {
            return new Open();
        }
    }

    public static class Arm {
        private final Servo armServo;

        public Arm(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class, "arm");
        }

        public class DropSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (currentArmPosition < Params.ARM_SAMPLE_POSITION) {
                    while (currentArmPosition < Params.ARM_SAMPLE_POSITION) {
                        if ((currentArmPosition + Params.ARM_INCREMENT) >= Params.ARM_SAMPLE_POSITION) {
                            currentArmPosition = Params.ARM_SAMPLE_POSITION;
                        } else {
                            currentArmPosition += Params.ARM_INCREMENT;
                        }
                        armServo.setPosition(currentArmPosition);
                    }
                } else {
                    while (currentArmPosition > Params.ARM_SAMPLE_POSITION) {
                        if ((currentArmPosition - Params.ARM_INCREMENT) <= Params.ARM_SAMPLE_POSITION) {
                            currentArmPosition = Params.ARM_SAMPLE_POSITION;
                        } else {
                            currentArmPosition -= Params.ARM_INCREMENT;
                        }
                        armServo.setPosition(currentArmPosition);
                    }
                }
                return false;
            }
        }

        public Action dropSample() {
            return new DropSample();
        }

        public class Lower implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                while (currentArmPosition < Params.MAX_ARM_POSITION) {
                    if ((currentArmPosition + Params.ARM_INCREMENT) >= Params.MAX_ARM_POSITION) {
                        currentArmPosition = Params.MAX_ARM_POSITION;
                    } else {
                        currentArmPosition += Params.ARM_INCREMENT;
                    }
                    armServo.setPosition(currentArmPosition);
                }
                return false;
            }
        }

        public Action lower() {
            return new Lower();
        }

        public class Rest implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(Params.MIN_ARM_POSITION);
                return false;
            }
        }

        public Action rest() {
            return new Rest();
        }
    }

    public static class LinearSlides {
        private final VoltageSensor voltageSensor;
        private final DcMotorEx spoolLeft, spoolRight;

        public LinearSlides(HardwareMap hardwareMap) {
            spoolLeft = hardwareMap.get(DcMotorEx.class, "spoolLeft");
            spoolRight = hardwareMap.get(DcMotorEx.class, "spoolRight");

            spoolRight.setDirection(DcMotorSimple.Direction.REVERSE);

            spoolLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spoolRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            spoolLeft.setTargetPosition(Params.MIN_SPOOL_POSITION);
            spoolRight.setTargetPosition(Params.MIN_SPOOL_POSITION);

            spoolLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spoolRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            spoolLeft.setPower(0.01);
            spoolRight.setPower(0.01);

            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        public class Raise implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentSpoolPosition = Params.SAMPLE_BASKET_POSITION;
                    initialized = true;
                }

                spoolLeft.setTargetPosition(currentSpoolPosition);
                spoolRight.setTargetPosition(currentSpoolPosition);

                double voltage = voltageSensor.getVoltage();
                double averagePID = ((controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition)
                        + controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition)) / 2) / voltage;
                averagePID = Math.floor(averagePID * 100) / 100;

                spoolLeft.setPower(averagePID);
                spoolRight.setPower(averagePID);

                return (averagePID > 0.01);
            }
        }

        public Action raise() {
            return new Raise();
        }

        public class Lower implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentSpoolPosition = Params.MIN_SPOOL_POSITION;
                    initialized = true;
                }

                spoolLeft.setTargetPosition(currentSpoolPosition);
                spoolRight.setTargetPosition(currentSpoolPosition);

                double voltage = voltageSensor.getVoltage();
                double averagePID = ((controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition)
                        + controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition)) / 2) / voltage;
                averagePID = Math.floor(averagePID * 100) / 100;

                spoolLeft.setPower(averagePID);
                spoolRight.setPower(averagePID);

                return (averagePID > 0.01);
            }
        }

        public Action lower() {
            return new Lower();
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
                .strafeToLinearHeading(new Vector2d(-55, -56), Math.toRadians(225))
                .stopAndAdd(linearSlides.raise())
                .stopAndAdd(arm.dropSample())
                .waitSeconds(0.5)
                .stopAndAdd(claw.open())
                .waitSeconds(0.5)
                .build();

        Action getSampleOne = drive.actionBuilder(new Pose2d(-55, -56, 0))
                .stopAndAdd(arm.rest())
                .stopAndAdd(linearSlides.lower())
                .strafeToLinearHeading(new Vector2d(-47.5, -45), Math.toRadians(90))
                .waitSeconds(1)
                .stopAndAdd(arm.lower())
                .waitSeconds(2)
                .stopAndAdd(claw.close())
                .waitSeconds(1)
                .stopAndAdd(arm.rest())
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-55, -56), Math.toRadians(225))
                .stopAndAdd(linearSlides.raise())
                .stopAndAdd(arm.dropSample())
                .waitSeconds(1)
                .stopAndAdd(claw.open())
                .waitSeconds(1)
                .build();

        Action getSampleTwo = drive.actionBuilder(new Pose2d(-55, -56, 0))
                .stopAndAdd(arm.rest())
                .waitSeconds(0.5)
                .stopAndAdd(linearSlides.lower())
                .strafeToLinearHeading(new Vector2d(-55, -58), Math.toRadians(90))
                .waitSeconds(1)
                .stopAndAdd(arm.lower())
                .waitSeconds(2)
                .stopAndAdd(claw.close())
                .waitSeconds(1)
                .stopAndAdd(arm.rest())
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-55, -56), Math.toRadians(225))
                .stopAndAdd(linearSlides.raise())
                .stopAndAdd(arm.dropSample())
                .waitSeconds(1)
                .stopAndAdd(claw.open())
                .waitSeconds(1)
                .build();

        Action getSampleThree = drive.actionBuilder(new Pose2d(-55, -56, 0))
                .stopAndAdd(arm.rest())
                .waitSeconds(0.5)
                .stopAndAdd(linearSlides.lower())
                .strafeToLinearHeading(new Vector2d(-49, -26), Math.toRadians(180))
                .waitSeconds(1)
                .stopAndAdd(arm.lower())
                .waitSeconds(2)
                .stopAndAdd(claw.close())
                .waitSeconds(1)
                .stopAndAdd(arm.rest())
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-55, -56), Math.toRadians(225))
                .stopAndAdd(linearSlides.raise())
                .stopAndAdd(arm.dropSample())
                .waitSeconds(1)
                .stopAndAdd(claw.open())
                .waitSeconds(1)
                .build();

        Action endAutonomous = drive.actionBuilder(initalStartingPose)
                .stopAndAdd(arm.rest())
                .waitSeconds(0.5)
                .stopAndAdd(linearSlides.lower())
                .waitSeconds(2)
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                        dropPreloadSample,
                        getSampleOne,
                        getSampleTwo,
                        getSampleThree,
                        endAutonomous
                )
        );
    }
}
