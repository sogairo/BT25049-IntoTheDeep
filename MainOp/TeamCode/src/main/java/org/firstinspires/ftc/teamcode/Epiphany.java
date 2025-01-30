package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "BT25046-Drive", group = "Competition")
public class Epiphany extends LinearOpMode {
    // Initialize Constants
    // ------------------------------------------------ //
    public static class Configuration {
        public double DRIVE_POWER_CONSTRAINT = 0.825;
        public double SPOOL_ASCENT_CONSTRAINT = 0.7;

        public int MAX_SPOOL_POSITION = 2000;
        public int MIN_SPOOL_POSITION = 0;
        public int SPOOL_INCREMENT = 30;
        public int SPOOL_POSITION_TOLERANCE = 7;
        public int SAMPLE_BASKET_POSITION = 2000;
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

    public static Epiphany.Configuration Params = new Epiphany.Configuration();

    public DcMotorEx topLeft, bottomLeft, bottomRight, topRight;
    public DcMotorEx spoolLeft, spoolRight;

    public Servo claw, pivot, armLeft, armRight;

    // Initialize Variables.
    // ------------------------------------------------ //
    public static int currentSpoolPosition = Params.MIN_SPOOL_POSITION;
    private int setPositionType = Params.SAMPLE_BASKET_POSITION;

    private double currentArmPosition = Params.MIN_ARM_POSITION;

    private boolean pressXDebounce = false;
    private boolean pressYDebounce = false;
    private boolean pressBDebounce = false;
    private boolean pressADebounce = false;

    // Set up PIDF Configuration
    // ------------------------------------------------ //
    public static class PIDFConfiguration {
        public double kP = 0.025;
        public double kI = 0.01;
        public double kD = 0; // NEVER TUNE D
        public double kF = 0.00001;
    }

    private final FtcDashboard dashboard =  FtcDashboard.getInstance();
    public static Epiphany.PIDFConfiguration PIDFParams = new Epiphany.PIDFConfiguration();
    private PIDFController controller;

    // Retrieve hardwareMap devices
    // ------------------------------------------------ //
    public void INITALIZE_HARDWARE() {
        // Drivetrain
        // ------------------------------------------------ //
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "bottomLeft");
        bottomRight = hardwareMap.get(DcMotorEx.class, "bottomRight");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        // ------------------------------------------------ //
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        armLeft = hardwareMap.get(Servo.class, "armLeft"); // Port 2
        armRight = hardwareMap.get(Servo.class, "armRight"); // Port 3

        claw.scaleRange(Params.MIN_CLAW_POSITION, Params.MAX_CLAW_POSITION);
        armRight.setDirection(Servo.Direction.REVERSE);

        armRight.setPosition(currentArmPosition);
        armLeft.setPosition(currentArmPosition);
        claw.setPosition(1);
        pivot.setPosition(1);

        // Linear Slides
        // ------------------------------------------------ //
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

    public void clawFunction() {
        // It literally opens and closes the claw... What do you expect?
        if (gamepad1.x && !pressXDebounce) {
            pressXDebounce = true;

            if (claw.getPosition() == 0) {
                claw.setPosition(1);
            } else {
                claw.setPosition(0);
            }
        } else if (!gamepad1.x && pressXDebounce) {
            pressXDebounce = false;
        }
    }

    public void pivotFunction() {
        // Rotates to claw depending on button press.
        // Make sure the Servo's 0 and 1 positions are programmed using a Servo programmer.
        if (gamepad1.a && !pressADebounce) {
            pressADebounce = true;

            if (pivot.getPosition() == 0) {
                pivot.setPosition(1);
            } else {
                pivot.setPosition(0);
            }
        } else if (!gamepad1.a && pressADebounce) {
            pressADebounce = false;
        }
    }

    public void armFunction() {
        // Moves the arm and has shortcuts binded to the dpad
        if (gamepad1.right_bumper) {
            if ((currentArmPosition + Params.ARM_INCREMENT) >= Params.MAX_ARM_POSITION) {
                currentArmPosition = Params.MAX_ARM_POSITION;
            } else {
                currentArmPosition += Params.ARM_INCREMENT;
            }
        } else if (gamepad1.left_bumper) {
            if ((currentArmPosition - Params.ARM_INCREMENT) <= Params.MIN_ARM_POSITION) {
                currentArmPosition = Params.MIN_ARM_POSITION;
            } else {
                currentArmPosition -= Params.ARM_INCREMENT;
            }
        }
        if (gamepad1.dpad_up) {
            if (setPositionType == Params.SPECIMEN_POSITION) {
                currentArmPosition = Params.ARM_SPECIMEN_POSITION;
            } else {
                currentArmPosition = Params.ARM_SAMPLE_POSITION;
            }
        } else if (gamepad1.dpad_left) {
            currentArmPosition = Params.ARM_GRAB_POSITION;
        } else if (gamepad1.dpad_right) {
            currentArmPosition = Params.MIN_ARM_POSITION;
        }

        armRight.setPosition(currentArmPosition);
        armLeft.setPosition(currentArmPosition);
    }

    public void spoolFunction() {
        // This portion is programmed with the thought of the linear slides
        // being in it's initial position ( all the way down ) with starting
        // the bot.
        controller.setPIDF(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

        if (gamepad1.right_trigger > 0) {
            if ((currentSpoolPosition + Params.SPOOL_INCREMENT) >= Params.MAX_SPOOL_POSITION) {
                currentSpoolPosition = Params.MAX_SPOOL_POSITION;
            } else {
                currentSpoolPosition += Params.SPOOL_INCREMENT;
            }
        } else if (gamepad1.left_trigger > 0) {
            if ((currentSpoolPosition - Params.SPOOL_INCREMENT) <= Params.MIN_SPOOL_POSITION) {
                currentSpoolPosition = Params.MIN_SPOOL_POSITION;
            } else {
                currentSpoolPosition -= Params.SPOOL_INCREMENT;
            }
        }
        if (gamepad1.b && !pressBDebounce) {
            pressBDebounce = true;
            if (setPositionType == Params.SPECIMEN_POSITION) {
                setPositionType = Params.SAMPLE_BASKET_POSITION;
            } else {
                setPositionType = Params.SPECIMEN_POSITION;
            }
        } else if (!gamepad1.b && pressBDebounce) {
            pressBDebounce = false;
        }
        if (gamepad1.y && !pressYDebounce) {
            pressYDebounce = true;
            if (currentSpoolPosition != setPositionType) {
                currentSpoolPosition = setPositionType;
                if (setPositionType == Params.SPECIMEN_POSITION) {
                    currentArmPosition = Params.ARM_SPECIMEN_POSITION;
                } else {
                    currentArmPosition = Params.ARM_SAMPLE_POSITION;
                }
            } else {
                currentSpoolPosition = Params.MIN_SPOOL_POSITION;
            }
        } else if (!gamepad1.y && pressYDebounce) {
            pressYDebounce = false;
        }

        spoolLeft.setTargetPosition(currentSpoolPosition);
        spoolRight.setTargetPosition(currentSpoolPosition);

        spoolLeft.setPower(Math.min(controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));
        spoolRight.setPower(Math.min(controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        INITALIZE_HARDWARE();

        TelemetryPacket packet = new TelemetryPacket();
        controller = new PIDFController(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            clawFunction();
            pivotFunction();
            armFunction();
            spoolFunction();
            spoolFunction();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double topLeftPower = ((y + x + rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT;
            double bottomLeftPower = ((y - x + rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT;
            double topRightPower = ((y - x - rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT;
            double bottomRightPower = ((y + x - rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT;

            topLeft.setPower(topLeftPower);
            bottomLeft.setPower(bottomLeftPower);
            topRight.setPower(topRightPower);
            bottomRight.setPower(bottomRightPower);

            /*
            // Telemetry
            // ------------------------------------------------ //
            telemetry.addData("Left 0 PID:", Math.min(controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));
            telemetry.addData("Right 1 PID:", Math.min(controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition), Params.SPOOL_ASCENT_CONSTRAINT));
            */

            if (setPositionType == Params.SPECIMEN_POSITION) {
                telemetry.addData("Toggled State (For LinearSlides)", "SPECIMEN");
            } else {
                telemetry.addData("Toggled State (For LinearSlides)", "SAMPLE");
            }
            telemetry.update();

            /*
            // Dashboard Packets
            // ------------------------------------------------ //
            packet.put("targetPosition", currentSpoolPosition);

            packet.put("Left 0", spoolLeft.getCurrentPosition());
            packet.put("Right 1", spoolRight.getCurrentPosition());
            */

            dashboard.sendTelemetryPacket(packet);
        }
    }
}