package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BT25046-Drive", group = "Competition")
public class UpdatedDriveProgram extends LinearOpMode {
    // PID Setup
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks_in_degree = 700 / 180.0;

    // Initialize Constants
    final double MOTOR_POWER_CONSTRAINT = 0.65;

    final double SPOOL_MOTOR_POWER = 1.0;
    final double STATIONARY_SPOOL_POWER = 0.0001; // Change this value if gravity is a higher force

    final int MAX_SPOOL_POSITION = 1620; // change later
    final int MIN_SPOOL_POSITION = 0; // change later
    final int SPOOL_INCREMENT = 10; // change later

    final double MAX_ARM_POSITION = 1;
    final double MIN_ARM_POSITION = 0;
    final double ARM_INCREMENT = 0.02; // change later

    // Initialize Variables
    int currentSpoolPosition = MIN_SPOOL_POSITION;
    double currentArmPosition = MIN_ARM_POSITION;

    boolean pressXDebounce = false;
    boolean pressYDebounce = false;

    @Override
    public void init() {

    }


    @Override
    public void runOpMode() throws InterruptedException {
        /*
        // Initialize Constants
        final double MOTOR_POWER_CONSTRAINT = 0.65;

        final double SPOOL_MOTOR_POWER = 1.0;
        final double STATIONARY_SPOOL_POWER = 0.0001; // Change this value if gravity is a higher force

        final int MAX_SPOOL_POSITION = 1620; // change later
        final int MIN_SPOOL_POSITION = 0; // change later
        final int SPOOL_INCREMENT = 10; // change later

        final double MAX_ARM_POSITION = 1;
        final double MIN_ARM_POSITION = 0;
        final double ARM_INCREMENT = 0.02; // change later
        */

        /*
        // Initialize Variables
        int currentSpoolPosition = MIN_SPOOL_POSITION;
        double currentArmPosition = MIN_ARM_POSITION;

        boolean pressXDebounce = false;
        boolean pressYDebounce = false;
        boolean pressBumperDebounce = false;
        */

        // Initialize Motors, Servos, Sensors

        // Drivetrain Configuration
        DcMotor topLeftMotor = hardwareMap.dcMotor.get("topLeft");
        DcMotor bottomLeftMotor = hardwareMap.dcMotor.get("bottomLeft");

        DcMotor topRightMotor = hardwareMap.dcMotor.get("topRight");
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor bottomRightMotor = hardwareMap.dcMotor.get("bottomRight");
        bottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Claw Configuration
        Servo remadeClawServo = hardwareMap.servo.get("remadeClaw");

        // Claw Pivot Configuration
        Servo clawPivot = hardwareMap.servo.get("clawPivot");

        // Arm Configuration
        Servo armRightServo = hardwareMap.servo.get("armRight");
        armRightServo.setDirection(Servo.Direction.REVERSE);

        Servo armLeftServo = hardwareMap.servo.get("armLeft");

        // Spool Linear Slides Configuration
        DcMotor spoolLeftMotor = hardwareMap.dcMotor.get("spoolLeft");
        spoolLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spoolLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spoolLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spoolLeftMotor.setTargetPosition(MIN_SPOOL_POSITION);
        spoolLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spoolLeftMotor.setPower(1);

        DcMotor spoolRightMotor = hardwareMap.dcMotor.get("spoolRight");
        spoolRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spoolRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spoolRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spoolRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spoolRightMotor.setTargetPosition(MIN_SPOOL_POSITION);
        spoolRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spoolRightMotor.setPower(1);

        // Yield until the program starts
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Claw Information
            //
            // If x is pressed, the claw will either open or close based on it's current state.

            if (gamepad1.x && !pressXDebounce) {
                pressXDebounce = true;

                if (remadeClawServo.getPosition() == 0) {
                    remadeClawServo.setPosition(1);
                } else {
                    remadeClawServo.setPosition(0);
                }
            } else if (!gamepad1.x && pressXDebounce) {
                pressXDebounce = false;
            }

            // --------------------------------------------------------------------

            // Claw Pivot Information
            //
            // If y is pressed, the claw pivot will pivot based on it's current state.

            if (gamepad1.y && !pressYDebounce) {
                pressYDebounce = true;

                if (clawPivot.getPosition() == 0) {
                    clawPivot.setPosition(1);
                } else {
                    clawPivot.setPosition(0);
                }
            } else if (!gamepad1.y && pressYDebounce) {
                pressYDebounce = false;
            }

            // --------------------------------------------------------------------

            // Linear Slides Spool Information
            //
            // If right trigger is held down, the linear slide will go up.
            // If the left trigger is held down, the linear slide will go down.
            // If none are pressed, the motor stays in it's current position, essentially stationary.

            spoolLeftMotor.setPower(SPOOL_MOTOR_POWER);
            spoolRightMotor.setPower(SPOOL_MOTOR_POWER);
            if (gamepad1.right_trigger > 0) {
                if ((currentSpoolPosition + SPOOL_INCREMENT) >= MAX_SPOOL_POSITION) {
                    currentSpoolPosition = MAX_SPOOL_POSITION;
                } else {
                    currentSpoolPosition += SPOOL_INCREMENT;
                }
                // might need to add if statements to set power to 0 when at current spool position
                spoolLeftMotor.setTargetPosition(currentSpoolPosition);
                spoolRightMotor.setTargetPosition(currentSpoolPosition);
            } else if (gamepad1.left_trigger > 0) {
                if ((currentSpoolPosition - SPOOL_INCREMENT) <= MIN_SPOOL_POSITION) {
                    currentSpoolPosition = MIN_SPOOL_POSITION;
                } else {
                    currentSpoolPosition -= SPOOL_INCREMENT;
                }

                spoolLeftMotor.setTargetPosition(currentSpoolPosition);
                spoolRightMotor.setTargetPosition(currentSpoolPosition);
            } else {
                spoolLeftMotor.setPower(STATIONARY_SPOOL_POWER);
                spoolRightMotor.setPower(STATIONARY_SPOOL_POWER);
            }

            // --------------------------------------------------------------------

            // Arm Information
            //
            // If right bumper is held down, the arm goes up.
            // If left bumper is held down, the arm goes down.
            // If none are pressed, the arm stays stationary.

            if (gamepad1.right_bumper) {
                if ((currentArmPosition + ARM_INCREMENT) >= MAX_ARM_POSITION) {
                    currentArmPosition = MAX_ARM_POSITION;
                } else {
                    currentArmPosition += ARM_INCREMENT;
                }
                armRightServo.setPosition(currentArmPosition);
                armLeftServo.setPosition(currentArmPosition);
            } else if (gamepad1.left_bumper) {
                if ((currentArmPosition - ARM_INCREMENT) <= MIN_ARM_POSITION) {
                    currentArmPosition = MIN_ARM_POSITION;
                } else {
                    currentArmPosition -= ARM_INCREMENT;
                }
                armRightServo.setPosition(currentArmPosition);
                armLeftServo.setPosition(currentArmPosition);
            }

            // --------------------------------------------------------------------

            // Retrieve gamepad stick information.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Cap the calculated motor power within a range of 1.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Calculate motor power allocation that is constraint by preset constant.
            double topLeftPower = ((y + x + rx) / denominator) * MOTOR_POWER_CONSTRAINT;
            double bottomLeftPower = ((y - x + rx) / denominator) * MOTOR_POWER_CONSTRAINT;
            double topRightPower = ((y - x - rx) / denominator) * MOTOR_POWER_CONSTRAINT;
            double bottomRightPower = ((y + x - rx) / denominator) * MOTOR_POWER_CONSTRAINT;

            // Set calculated motor power into the motors.
            topLeftMotor.setPower(topLeftPower);
            bottomLeftMotor.setPower(bottomLeftPower);
            topRightMotor.setPower(topRightPower);
            bottomRightMotor.setPower(bottomRightPower);

            // Telemetry data
            /*
            telemetry.addData("GamepadX", gamepad1.x);
            telemetry.addData("Opened", remadeClawServo.getPosition() == 0);
            telemetry.addData("Position", remadeClawServo.getPosition());
            telemetry.addData("------------", "------------");
            */

            telemetry.addData("RightBumper", gamepad1.right_bumper);
            telemetry.addData("LeftBumper", gamepad1.left_bumper);
            telemetry.addData("currentArmPosition", currentArmPosition);
            telemetry.addData("Servo2 Position", armLeftServo.getPosition());
            telemetry.addData("Servo3 Position", armRightServo.getPosition());
            telemetry.addData("------------", "------------");

            telemetry.addData("RightTrigger", gamepad1.right_trigger > 0);
            telemetry.addData("LeftTrigger", gamepad1.left_trigger > 0);
            telemetry.addData("currentSpoolPosition", currentSpoolPosition);
            telemetry.addData("Motor0 Position", spoolLeftMotor.getCurrentPosition());
            telemetry.addData("Motor1 Position", spoolRightMotor.getCurrentPosition());
            telemetry.addData("------------", "------------");

            telemetry.update();
        }
    }
}