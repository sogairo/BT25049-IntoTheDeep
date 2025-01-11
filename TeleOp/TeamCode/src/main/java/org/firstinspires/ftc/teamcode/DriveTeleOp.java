package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BT25046-Drive", group = "Competition")
public class DriveTeleOp extends LinearOpMode {
    // On Initialize
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Constants
        final double DRIVE_POWER_CONSTRAINT = 0.8;

        final int MAX_SPOOL_POSITION = 1610;
        final int MIN_SPOOL_POSITION = 0;
        final int SPOOL_INCREMENT = 30;
        final int SAMPLE_BASKET_POSITION = 1610;
        final int SPECIMEN_POSITION = 800;

        final double MAX_ARM_POSITION = 0.96;
        final double MID_ARM_POSITION = 0.36;
        final double MIN_ARM_POSITION = 0;
        final double ARM_INCREMENT = 0.02;

        // Initialize Variables.
        int currentSpoolPosition = MIN_SPOOL_POSITION;
        int setPositionType = SAMPLE_BASKET_POSITION;

        double currentArmPosition = MIN_ARM_POSITION;

        boolean pressXDebounce = false;
        boolean pressYDebounce = false;
        boolean pressBDebounce = false;
        boolean pressADebounce = false;

        // PID Setup
        PIDController controller;
        double p = 0.004, i = 0.2, d = 0.02;

        controller = new PIDController(p, i ,d);

        // Drivetrain Configuration.
        DcMotor topLeftMotor = hardwareMap.dcMotor.get("topLeft");
        DcMotor bottomLeftMotor = hardwareMap.dcMotor.get("bottomLeft");

        DcMotor topRightMotor = hardwareMap.dcMotor.get("topRight");
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor bottomRightMotor = hardwareMap.dcMotor.get("bottomRight");
        bottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Claw Configuration.
        Servo remadeClawServo = hardwareMap.servo.get("remadeClaw");
        remadeClawServo.setPosition(0);

        // Claw Pivot Configuration.
        Servo clawPivot = hardwareMap.servo.get("clawPivot");
        clawPivot.setPosition(0);

        // Arm Configuration.
        Servo armRightServo = hardwareMap.servo.get("armRight");
        armRightServo.setDirection(Servo.Direction.REVERSE);
        armRightServo.setPosition(currentArmPosition);

        Servo armLeftServo = hardwareMap.servo.get("armLeft");
        armLeftServo.setPosition(currentArmPosition);

        // Spool Linear Slides Configuration.
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

        // Yield until the program starts.
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Claw Information
            //
            // It literally opens and closes the claw... What do you expect?

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
            // Rotates to claw depending on button press.
            //
            // Make sure the Servo's 0 and 1 positions are programmed using a
            // Servo programmer.

            if (gamepad1.a && !pressADebounce) {
                pressADebounce = true;

                if (clawPivot.getPosition() == 0) {
                    clawPivot.setPosition(1);
                } else {
                    clawPivot.setPosition(0);
                }
            } else if (!gamepad1.a && pressADebounce) {
                pressADebounce = false;
            }

            // --------------------------------------------------------------------

            // Linear Slides Spool Information
            //
            // This portion is programmed with the thought of the linear slides
            // being in it's initial position ( all the way down ) with starting
            // the bot.

            // manual linear slide movement
            controller.setPID(p, i, d);
            if (gamepad1.right_trigger > 0) {
                if ((currentSpoolPosition + SPOOL_INCREMENT) >= MAX_SPOOL_POSITION) {
                    currentSpoolPosition = MAX_SPOOL_POSITION;
                } else {
                    currentSpoolPosition += SPOOL_INCREMENT;
                }
            } else if (gamepad1.left_trigger > 0) {
                if ((currentSpoolPosition - SPOOL_INCREMENT) <= MIN_SPOOL_POSITION) {
                    currentSpoolPosition = MIN_SPOOL_POSITION;
                } else {
                    currentSpoolPosition -= SPOOL_INCREMENT;
                }
            }

            // toggle
            if (gamepad1.b && !pressBDebounce) {
                pressBDebounce = true;
                if (setPositionType == SPECIMEN_POSITION) {
                    setPositionType = SAMPLE_BASKET_POSITION;
                } else {
                    setPositionType = SPECIMEN_POSITION;
                }
            } else if (!gamepad1.b && pressBDebounce) {
                pressBDebounce = false;
            }

            // shortcut to sample/specimen position
            if (gamepad1.y && !pressYDebounce) {
                pressYDebounce = true;
                if (currentSpoolPosition != setPositionType) {
                    currentSpoolPosition = setPositionType;
                } else {
                    currentSpoolPosition = MIN_SPOOL_POSITION;
                }
            } else if (!gamepad1.y && pressYDebounce) {
                pressYDebounce = false;
            }

            spoolLeftMotor.setPower(controller.calculate(spoolLeftMotor.getCurrentPosition(), currentSpoolPosition));
            spoolLeftMotor.setTargetPosition(currentSpoolPosition);

            spoolRightMotor.setPower(controller.calculate(spoolRightMotor.getCurrentPosition(), currentSpoolPosition));
            spoolRightMotor.setTargetPosition(currentSpoolPosition);

            // --------------------------------------------------------------------

            // Arm Information
            //
            // The servo still tweaks out a bit when going to it's position
            // but I guess it's fine?
            //
            // Too bad that PIDs don't work/exist for Servos... or do they...?

            // manual
            if (gamepad1.right_bumper) {
                if ((currentArmPosition + ARM_INCREMENT) >= MAX_ARM_POSITION) {
                    currentArmPosition = MAX_ARM_POSITION;
                } else {
                    currentArmPosition += ARM_INCREMENT;
                }
            } else if (gamepad1.left_bumper) {
                if ((currentArmPosition - ARM_INCREMENT) <= MIN_ARM_POSITION) {
                    currentArmPosition = MIN_ARM_POSITION;
                } else {
                    currentArmPosition -= ARM_INCREMENT;
                }
            }

            // shortcuts
            if (gamepad1.dpad_up) {
                currentArmPosition = MID_ARM_POSITION;
            } else if (gamepad1.dpad_left) {
                currentArmPosition = MAX_ARM_POSITION;
            } else if (gamepad1.dpad_right) {
                currentArmPosition = MIN_ARM_POSITION;
            }

            armRightServo.setPosition(currentArmPosition);
            armLeftServo.setPosition(currentArmPosition);

            // --------------------------------------------------------------------

            // Retrieve gamepad stick information.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Cap the calculated motor power within a range of 1.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Calculate motor power allocation that is constraint by preset constant.
            double topLeftPower = ((y + x + rx) / denominator) * DRIVE_POWER_CONSTRAINT;
            double bottomLeftPower = ((y - x + rx) / denominator) * DRIVE_POWER_CONSTRAINT;
            double topRightPower = ((y - x - rx) / denominator) * DRIVE_POWER_CONSTRAINT;
            double bottomRightPower = ((y + x - rx) / denominator) * DRIVE_POWER_CONSTRAINT;

            // Set calculated motor power into the motors.
            topLeftMotor.setPower(topLeftPower);
            bottomLeftMotor.setPower(bottomLeftPower);
            topRightMotor.setPower(topRightPower);
            bottomRightMotor.setPower(bottomRightPower);

            // --------------------------------------------------------------------

            // Telemetry data
            //
            // Outputs the data of each variable

            telemetry.addData("GamepadX", gamepad1.x);
            telemetry.addData("Opened", remadeClawServo.getPosition() == 0);
            telemetry.addData("Position", remadeClawServo.getPosition());
            telemetry.addData("------------", "------------");

            telemetry.addData("RightBumper", gamepad1.right_bumper);
            telemetry.addData("LeftBumper", gamepad1.left_bumper);
            telemetry.addData("currentArmPosition", currentArmPosition);
            telemetry.addData("Servo2 Position", armLeftServo.getPosition());
            telemetry.addData("Servo3 Position", armRightServo.getPosition());
            telemetry.addData("------------", "------------");

            if (setPositionType == SPECIMEN_POSITION) {
                telemetry.addData("Toggled State", "SPECIMEN");
            } else {
                telemetry.addData("Toggled State", "SAMPLE");
            }
            telemetry.addData("RightTrigger", gamepad1.right_trigger > 0);
            telemetry.addData("LeftTrigger", gamepad1.left_trigger > 0);
            telemetry.addData("currentSpoolPosition", currentSpoolPosition);
            telemetry.addData("Motor0 Position", spoolLeftMotor.getCurrentPosition());
            telemetry.addData("Motor1 Position", spoolRightMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}