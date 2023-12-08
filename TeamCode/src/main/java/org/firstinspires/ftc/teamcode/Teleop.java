package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Normal")
public class Teleop extends OpMode {

    private Drivetrain mecanum;
    private DcMotorEx motorA, motorB;
    private Servo claw1, claw2, wrist;

    public static int currentPosition = 0;

    private int armSetpoint = 400;
    private int armIncrement = 50;

    // Enum for different arm states
    private enum ArmState {
        MANUAL_CONTROL,
        ARM_UP,
        ARM_DOWN
    }

    private ArmState currentArmState = ArmState.MANUAL_CONTROL;

    // PID constants for arm control
    public static double armP = 0.1;
    private static double armI = 0.01;
    private static double armD = 0.001;
    // Variables for PID control
    private double previousError = 0;
    private double integral = 0;

    @Override
    public void init() {
        mecanum = new Drivetrain(gamepad1, hardwareMap);
        motorA = hardwareMap.get(DcMotorEx.class, "A1");
        motorB = hardwareMap.get(DcMotorEx.class, "A2");
        claw1 = hardwareMap.get(Servo.class, "leftPixel");
        claw2 = hardwareMap.get(Servo.class, "rightPixel");
        wrist = hardwareMap.get(Servo.class, "wrist");

        motorB.setDirection(DcMotor.Direction.REVERSE);

        setupMotor(motorA);
        setupMotor(motorB);

        resetArm();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        mecanum.drive();

        handleArmControls();
        updateArmPosition();

        handleClawControls();

        telemetry.addData("Encoder PositionA", motorA.getCurrentPosition());
        telemetry.addData("Encoder PositionB", motorB.getCurrentPosition());
        telemetry.addData("Claw Position", claw1.getPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.update();
    }

    private void setupMotor(DcMotorEx motor) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void handleArmControls() {
        switch (currentArmState) {
            case MANUAL_CONTROL:
                // Allow manual control of the arm
                currentPosition += (int) (gamepad1.left_stick_y * armIncrement);
                break;
            case ARM_UP:
                // Use PID control to move the arm to the desired setpoint
                pidControlArm();
                break;
            case ARM_DOWN:
                // Use PID control to move the arm to the desired setpoint
                pidControlArm();
                break;
        }

        // Switch arm state based on button inputs
        if (gamepad1.dpad_up) {
            currentArmState = ArmState.ARM_UP;
        } else if (gamepad1.dpad_down) {
            currentArmState = ArmState.ARM_DOWN;
        } else {
            currentArmState = ArmState.MANUAL_CONTROL;
        }
    }

    private void updateArmPosition() {
        currentPosition = Math.min(Math.max(0, currentPosition), 700);

        motorA.setTargetPosition(currentPosition);
        motorB.setTargetPosition(currentPosition);

        motorA.setPower(0.75);
        motorB.setPower(0.75);
    }

    private void pidControlArm() {
        int error = armSetpoint - motorA.getCurrentPosition();
        integral += error;
        double derivative = error - previousError;

        double output = armP * error + armI * integral + armD * derivative;

        currentPosition += output;

        previousError = error;
    }

    private void handleClawControls() {
        if (gamepad1.left_trigger > 0) {
            openClaw();
        } else if (gamepad1.right_trigger > 0) {
            closeClaw();
        }
    }

    private void armUp() {
        currentArmState = ArmState.ARM_UP;
        wrist.setPosition(Constants.wristStowOrOutTake);
    }

    private void armDown() {
        currentArmState = ArmState.ARM_DOWN;
        wrist.setPosition(Constants.wristIntake);
    }

    private void openClaw() {
        setClawPosition(0.2);
    }

    private void closeClaw() {
        setClawPosition(0.4);
    }

    private void setClawPosition(double position) {
        claw1.setPosition(position);
        claw2.setPosition(position);
    }

    private void resetArm() {
        currentPosition = Constants.startingArmPos;
    }
}