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
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(name = "BT25046-Drive", group = "Competition")
public class Epiphany extends LinearOpMode {
    // TODO: Retune the MAX_SPOOL_POSITION.

    // Initialize Constants
    // ------------------------------------------------ //
    public static class Configuration {
        public boolean SHOW_DEBUG = false;

        public double DRIVE_POWER_CONSTRAINT = 0.85;

        public int MAX_SPOOL_POSITION = 2000;
        public int MIN_SPOOL_POSITION = 0;
        public int SPOOL_POSITION_TOLERANCE = 7;
        public int SPOOL_INCREMENT = 30;

        public int SAMPLE_BASKET_POSITION = 2000;
        public int SPECIMEN_POSITION = 615;

        public double MAX_ARM_POSITION = 0.985;
        public double MIN_ARM_POSITION = 0;
        public double ARM_INCREMENT = 0.04;

        public double ARM_SAMPLE_POSITION = 0.5;
        public double ARM_SPECIMEN_POSITION = 0.48;
        public double ARM_GRAB_POSITION = 0.88;

        public double MIN_CLAW_POSITION = 0.455;
        public double MAX_CLAW_POSITION = 0.625;
    }

    public static Epiphany.Configuration Params = new Epiphany.Configuration();

    // Initialize Hardware Values
    // ------------------------------------------------ //
    public VoltageSensor voltageSensor;

    public TelemetryPacket packet;

    public DcMotorEx topLeft, bottomLeft, bottomRight, topRight;
    public DcMotorEx spoolLeft, spoolRight;

    public Servo claw, pivot, arm;

    // Initialize Variables
    // ------------------------------------------------ //
    public static int currentSpoolPosition = Params.MIN_SPOOL_POSITION;
    private int setPositionType = Params.SAMPLE_BASKET_POSITION;

    private double currentArmPosition = Params.MIN_ARM_POSITION;

    private boolean pressXDebounce = false;
    private boolean pressYDebounce = false;
    private boolean pressBDebounce = false;
    private boolean pressADebounce = false;

    // Setup PIDF Configuration
    // ------------------------------------------------ //
    public static class PIDFConfiguration {
        public double kP = 0.025;
        public double kI = 0.01;
        public double kD = 0;
        public double kF = 0.00001;
    }

    public static Epiphany.PIDFConfiguration PIDFParams = new Epiphany.PIDFConfiguration();
    private PIDFController controller;

    // Initialize Dashboard
    // ------------------------------------------------ //
    private final FtcDashboard dashboard =  FtcDashboard.getInstance();

    // Initialize Hardware Function
    // ------------------------------------------------ //
    public void initializeHardware() {
        // Miscellaneous
        // ------------------------------------------------ //
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        controller = new PIDFController(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);
        packet = new TelemetryPacket();

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

        // Servos
        // ------------------------------------------------ //
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        arm = hardwareMap.get(Servo.class, "arm");

        claw.scaleRange(Params.MIN_CLAW_POSITION, Params.MAX_CLAW_POSITION);
        arm.setDirection(Servo.Direction.REVERSE);

        claw.setPosition(1);
        pivot.setPosition(1);

        // Linear Slides
        // ------------------------------------------------ //
        spoolLeft = hardwareMap.get(DcMotorEx.class, "spoolLeft");
        spoolRight = hardwareMap.get(DcMotorEx.class, "spoolRight");

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

    // Claw Function
    // ------------------------------------------------ //
    public void clawFunction() {
        // Manual
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

        // Debug
        // ------------------------------------------------ //
        if (Params.SHOW_DEBUG) {
            telemetry.addData("Claw Position", claw.getPosition());
        }
    }

    // Pivot Function
    // ------------------------------------------------ //
    public void pivotFunction() {
        // Manual
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

        // Debug
        // ------------------------------------------------ //
        if (Params.SHOW_DEBUG) {
            telemetry.addData("Pivot Position", pivot.getPosition());
        }
    }

    // Arm Function
    // ------------------------------------------------ //
    public void armFunction() {
        // Manual
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

        // Hotkey
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

        // Debug
        // ------------------------------------------------ //
        if (Params.SHOW_DEBUG) {
            telemetry.addData("Arm Position", currentArmPosition);
        }

        arm.setPosition(currentArmPosition);
    }

    // Spool Function
    // ------------------------------------------------ //
    public void spoolFunction() {
        controller.setPIDF(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

        // Override
        if (gamepad1.dpad_down) {
            currentSpoolPosition -= Params.SPOOL_INCREMENT;
        }

        // Manual
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

        // Toggle
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

        // Hotkey
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

        double voltage = voltageSensor.getVoltage();
        double averagePID = ((controller.calculate(spoolLeft.getCurrentPosition(), currentSpoolPosition)
                + controller.calculate(spoolRight.getCurrentPosition(), currentSpoolPosition)) / 2) / voltage;

        // Debug
        // ------------------------------------------------ //
        if (Params.SHOW_DEBUG) {
            packet.put("averagePID", averagePID);

            packet.put("targetPosition", currentSpoolPosition);

            packet.put("Left 0", spoolLeft.getCurrentPosition());
            packet.put("Right 1", spoolRight.getCurrentPosition());
        }

        spoolLeft.setPower(averagePID);
        spoolRight.setPower(averagePID);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        // Dashboard Setup
        // ------------------------------------------------ //
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            clawFunction();
            pivotFunction();
            armFunction();
            spoolFunction();
            spoolFunction();

            // Drive
            // ------------------------------------------------ //
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double topLeftPower = (((y + x + rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT);
            double bottomLeftPower = (((y - x + rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT);
            double topRightPower = (((y - x - rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT);
            double bottomRightPower = (((y + x - rx) / denominator) * Params.DRIVE_POWER_CONSTRAINT);

            topLeft.setPower(topLeftPower);
            bottomLeft.setPower(bottomLeftPower);
            topRight.setPower(topRightPower);
            bottomRight.setPower(bottomRightPower);

            // Competition Telemetry
            // ------------------------------------------------ //
            if (setPositionType == Params.SPECIMEN_POSITION) {
                telemetry.addData("Toggled State (For LinearSlides)", "SPECIMEN");
            } else {
                telemetry.addData("Toggled State (For LinearSlides)", "SAMPLE");
            }
            telemetry.update();

            // Debug Dashboard Packets
            // ------------------------------------------------ //
            if (Params.SHOW_DEBUG) {
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}