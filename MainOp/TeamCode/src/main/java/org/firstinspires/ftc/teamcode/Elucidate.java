/**
 *
 * @author sogairo
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
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

                    TelemetryPacket temp = new TelemetryPacket();
                    temp.put("CurrentArmPosition", currentArmPosition);
                    FtcDashboard.getInstance().sendTelemetryPacket(temp);

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

                if (averagePID <= 0.5) {
                    currentArmPosition = Params.ARM_SAMPLE_POSITION;
                    armServo.setPosition(currentArmPosition);
                }

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

                return (averagePID > 0.04);
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
        Pose2d initialStartingPose = new Pose2d(-38, -61.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialStartingPose);

        Claw claw = new Claw(hardwareMap);
        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        Action dropPreloadSample = drive.actionBuilder(initialStartingPose)
                .stopAndAdd(arm.rest())
                .stopAndAdd(pivot.horizontal())

                // Drop Sample
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(initialStartingPose)
                                .strafeToLinearHeading(new Vector2d(-55, -56), Math.toRadians(225))
                                .build(),
                        linearSlides.raise()
                ))

                .waitSeconds(0.5)
                .stopAndAdd(claw.open())
                .waitSeconds(0.2)
                .build();

        Action getSampleOne = drive.actionBuilder(new Pose2d(-55, -56, Math.toRadians(225)))
                .stopAndAdd(new ParallelAction(
                        arm.rest(),
                        linearSlides.lower()
                ))
                .strafeToLinearHeading(new Vector2d(-48.1, -45), Math.toRadians(90)) // TODO: INACCURATE -----------------------------
                .waitSeconds(0.4)

                // Grab Sample
                .stopAndAdd(arm.lower())
                .waitSeconds(0.2)
                .stopAndAdd(claw.close())
                
                .waitSeconds(0.2)

                // Reset Arm
                .stopAndAdd(arm.rest())

                // Drop Sample
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(-48.1, -45, Math.toRadians(90)))
                                .strafeToLinearHeading(new Vector2d(-55, -56), Math.toRadians(225))
                                .build(),
                        linearSlides.raise()
                ))

                .waitSeconds(0.5)
                .stopAndAdd(claw.open())
                .waitSeconds(0.2)
                .build();

        Action getSampleTwo = drive.actionBuilder(new Pose2d(-55, -56, Math.toRadians(225)))
                .stopAndAdd(new ParallelAction(
                        arm.rest(),
                        linearSlides.lower()
                ))
                .strafeToLinearHeading(new Vector2d(-59.2, -46), Math.toRadians(95))  // TODO: INACCURATE -----------------------------
                .waitSeconds(0.4)

                // Grab Sample
                .stopAndAdd(arm.lower())
                .waitSeconds(0.2)
                .stopAndAdd(claw.close())
                .waitSeconds(0.2)

                // Reset Arm
                .stopAndAdd(arm.rest())

                // Drop Sample
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(-59.2, -46, Math.toRadians(95)))
                                .waitSeconds(0.3)
                                .strafeToLinearHeading(new Vector2d(-55, -56), Math.toRadians(225))
                                .build(),
                        linearSlides.raise()
                ))

                .waitSeconds(0.5)
                .stopAndAdd(claw.open())
                .waitSeconds(0.2)
                .build();

        Action getSampleThree = drive.actionBuilder(new Pose2d(-55, -56, Math.toRadians(225)))
                .stopAndAdd(pivot.vertical())
                .stopAndAdd(new ParallelAction(
                        arm.rest(),
                        linearSlides.lower()
                ))
                .strafeToLinearHeading(new Vector2d(-51.2, -25), Math.toRadians(190))  // TODO: INACCURATE -----------------------------
                //.waitSeconds(0.4)

                // Grab sample
                .stopAndAdd(arm.lower())
                .waitSeconds(0.2)
                .stopAndAdd(claw.close())
                .waitSeconds(0.2)

                // Reset arm
                .stopAndAdd(arm.rest())

                // Drop Sample
                .stopAndAdd(new ParallelAction(
                        drive.actionBuilder(new Pose2d(-51.2, -25, Math.toRadians(190)))
                                .strafeToLinearHeading(new Vector2d(-53, -57), Math.toRadians(230))
                                .build(),
                        linearSlides.raise()
                ))

                .waitSeconds(0.5)
                .stopAndAdd(claw.open())
                .waitSeconds(0.2)
                .build();

        Action endAutonomous = drive.actionBuilder(initialStartingPose)
                .stopAndAdd(arm.rest())
                .stopAndAdd(linearSlides.lower())
                .strafeToLinearHeading(new Vector2d(-50, -32), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-24.5, -10), 0)
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
