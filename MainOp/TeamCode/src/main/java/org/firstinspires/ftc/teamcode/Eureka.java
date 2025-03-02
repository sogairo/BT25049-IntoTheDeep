/**
 *
 * @author sogairo
 */

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
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    private static double currentArmPosition = Params.MIN_ARM_POSITION;


    // Claw Action
    // ------------------------------------------------ //
    public static class Claw {
        private final Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "claw");
            clawServo.scaleRange(Params.MIN_CLAW_POSITION, Params.MAX_CLAW_POSITION);
            clawServo.setPosition(1);
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(1);
                return false;
            }
        }

        public Action close() {
            return new Close();
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0);
                return false;
            }
        }

        public Action open() {
            return new Open();
        }
    }

    // Pivot Action
    // ------------------------------------------------ //
    public static class Pivot {
        private final Servo pivotServo;

        public Pivot(HardwareMap hardwareMap) {
            pivotServo = hardwareMap.get(Servo.class, "pivot");
            pivotServo.scaleRange(Params.MIN_PIVOT_POSITION, Params.MAX_PIVOT_POSITION);
            pivotServo.setPosition(0);
        }

        public class Horizontal implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivotServo.setPosition(0);
                return false;
            }
        }

        public Action horizontal() {
            return new Horizontal();
        }

        public class Vertical implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivotServo.setPosition(1);
                return false;
            }
        }

        public Action vertical() {
            return new Vertical();
        }
    }

    // Arm Action
    // ------------------------------------------------ //
    public static class Arm {
        private final Servo armServo;

        public Arm(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class, "arm");
        }

        public class Lower implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                while (currentArmPosition < Params.MAX_ARM_POSITION) {
                    if ((currentArmPosition + Params.ARM_AUTONOMOUS_INCREMENT) >= Params.MAX_ARM_POSITION) {
                        currentArmPosition = Params.MAX_ARM_POSITION;
                        break;
                    } else {
                        currentArmPosition += Params.ARM_AUTONOMOUS_INCREMENT;
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
                currentArmPosition = Params.MIN_ARM_POSITION;
                armServo.setPosition(currentArmPosition);
                return false;
            }
        }

        public Action rest() {
            return new Rest();
        }
    }

    // Linear Slides Action
    // ------------------------------------------------ //
    public static class LinearSlides {
        private final VoltageSensor voltageSensor;
        private final DcMotorEx spoolLeft, spoolRight;

        private final Servo armServo;

        public LinearSlides(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class, "arm");

            spoolLeft = hardwareMap.get(DcMotorEx.class, "spoolLeft");
            spoolRight = hardwareMap.get(DcMotorEx.class, "spoolRight");

            spoolLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
                    currentSpoolPosition = Params.SPECIMEN_POSITION;
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

                if (averagePID <= 0.5) {
                    currentArmPosition = Params.ARM_SPECIMEN_POSITION;
                    armServo.setPosition(currentArmPosition);
                }
                return ((spoolLeft.getCurrentPosition() < currentSpoolPosition - 15) && (spoolRight.getCurrentPosition() < currentSpoolPosition - 15));
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

                if ((spoolLeft.getCurrentPosition() < currentSpoolPosition + 15) || (spoolRight.getCurrentPosition() < currentSpoolPosition + 15)) {
                    spoolRight.setPower(0);
                    spoolLeft.setPower(0);
                    return false;
                }
                return true;
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
        Pose2d initialStartingPose = new Pose2d(0, -61.5,  Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialStartingPose);

        Claw claw = new Claw(hardwareMap);
        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        Action hangPreload = drive.actionBuilder(initialStartingPose)
                .stopAndAdd(arm.rest())
                .stopAndAdd(pivot.horizontal())

                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(initialStartingPose)
                                .lineToY(-34)
                                .build(),
                        linearSlides.raise()
                ))
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(initialStartingPose)
                                .waitSeconds(0.2)
                                .stopAndAdd(claw.open())
                                .build(),
                        linearSlides.lower()
                ))
                .build();

        Action pushSamples = drive.actionBuilder(new Pose2d(0, -34, Math.toRadians(90)))
                // away
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(0, -34, Math.toRadians(90)))
                                .strafeToLinearHeading(new Vector2d(20, -40), Math.toRadians(-90))
                                .build(),
                        linearSlides.lower(),
                        arm.rest()
                ))

                // sample 1 push
                .splineToConstantHeading(new Vector2d(39, -9), Math.PI / 3)
                .splineToConstantHeading(new Vector2d(48, -45), Math.PI / 2)

                // sample 2 push
                .splineToConstantHeading(new Vector2d(50, -9), Math.PI / 3)
                .splineToConstantHeading(new Vector2d(58, -45), Math.PI / 2)

                // sample 3 push
                .splineToConstantHeading(new Vector2d(57, -9), Math.PI / 3)
                .splineToConstantHeading(new Vector2d(59, -45), Math.PI / 2)

                // load claw
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(59, -45, Math.toRadians(-90)))
                                .splineToLinearHeading(new Pose2d(23, -61.5, Math.toRadians(0)), -Math.PI / 2)
                                .build(),
                        arm.lower()
                ))
                .waitSeconds(0.2)
                .stopAndAdd(claw.close())
                .build();

        Action hangSpecimens = drive.actionBuilder(new Pose2d(23, -61.5, Math.toRadians(0)))
                // specimen 1
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(23, -61.5, Math.toRadians(0)))
                                .strafeToLinearHeading(new Vector2d(1, -33), Math.toRadians(90))
                                .build(),
                        linearSlides.raise()
                ))

                // put on rack
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(1, -33, Math.toRadians(90)))
                                .waitSeconds(0.2)
                                .stopAndAdd(claw.open())
                                .build(),
                        linearSlides.lower()
                ))

                // grab another specimen
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(1, -33, Math.toRadians(90)))
                                .strafeToLinearHeading(new Vector2d(23, -61.5), Math.toRadians(0))
                                .build(),
                        arm.lower()
                ))
                .waitSeconds(0.2)
                .stopAndAdd(claw.close())

                // specimen 2
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(23, -61.5, Math.toRadians(0)))
                                .strafeToLinearHeading(new Vector2d(2, -33), Math.toRadians(90))
                                .build(),
                        linearSlides.raise()
                ))

                // put on rack
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(2, -33, Math.toRadians(90)))
                                .waitSeconds(0.2)
                                .stopAndAdd(claw.open())
                                .build(),
                        linearSlides.lower()
                ))
                .build();

        Action endAutonomous = drive.actionBuilder(new Pose2d(2, -33, Math.toRadians(90)))
                .stopAndAdd(new ParallelAction(
                        arm.rest(),
                        drive.actionBuilder(new Pose2d(2, -33, Math.toRadians(90)))
                                .strafeToConstantHeading(new Vector2d(56, -61.5))
                                .build()
                ))
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                        hangPreload,
                        pushSamples,
                        hangSpecimens,
                        endAutonomous
                )
        );
    }
}
