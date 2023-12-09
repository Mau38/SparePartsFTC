package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ARM_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.Constants.ARM_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.Constants.armClimbPosition;
import static org.firstinspires.ftc.teamcode.Constants.armIntake;
import static org.firstinspires.ftc.teamcode.Constants.armScorePosition;
import static org.firstinspires.ftc.teamcode.Constants.revHubOrientation;
import static org.firstinspires.ftc.teamcode.Constants.startingArmPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "CompTeleOp", group = "Comp")
public class Teleop extends OpMode {

    private Drivetrain mecanum;

    public static boolean USING_ARM = true;
    private DcMotorEx motorA, motorB;
    private Servo claw1, claw2, wrist;

    public static int currentPosition = startingArmPos;

    private int armSetpoint = 400;
    private int armIncrement = 50;
    private IMU imu;

    // Enum for different arm states
    private enum ArmState {
        MANUAL_CONTROL,
        ARM_UP,
        ARM_DOWN,
        ARM_SCORE,
        ARM_CLIMB
    }

    private ArmState currentArmState = ArmState.MANUAL_CONTROL;

    // PID constants for arm control
    public static double armP = 0.1;
    private static double armI = 0.01;
    private static double armD = 0.001;
    // Variables for PID control
    private double previousError = 0;
    private double integral = 0;

    public static double wristOuttake = .2;
    public static double wristIntake = -.2;

    @Override
    public void init() {
        mecanum = new Drivetrain(gamepad1, hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                revHubOrientation
        );
        imu.initialize(parameters);
        imu.resetYaw();

        if (USING_ARM) {
            motorA = hardwareMap.get(DcMotorEx.class, "A1");
            motorB = hardwareMap.get(DcMotorEx.class, "A2");
            claw1 = hardwareMap.get(Servo.class, "leftPixel");
            claw2 = hardwareMap.get(Servo.class, "rightPixel");
            wrist = hardwareMap.get(Servo.class, "wrist");

            motorB.setDirection(DcMotor.Direction.REVERSE);

            setupMotor(motorA);
            setupMotor(motorB);

            resetArm();
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        //mecanum.drive();
        if (gamepad1.back) {
            imu.resetYaw();
        }

        mecanum.drive(imu);
        telemetry.addData("YAW", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        if (USING_ARM) {
            handleArmControls();
            updateArmPosition();
            handleArmControls();

            handleClawControls();

            IntakeMode();

            telemetry.addData("Encoder PositionA", motorA.getCurrentPosition());
            telemetry.addData("Encoder PositionB", motorB.getCurrentPosition());
            telemetry.addData("Claw Position", claw1.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
        }

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
                currentPosition += (int) (gamepad1.left_trigger * armIncrement);
                break;
            case ARM_UP:
                // Use PID control to move the arm to the desired setpoint
                pidControlArm();
                break;
            case ARM_DOWN:
                // Use PID control to move the arm to the desired setpoint
                pidControlArm();
                break;
            case ARM_CLIMB:
                currentPosition = armClimbPosition;
                break;
            case ARM_SCORE:
                currentPosition = armScorePosition;
                break;
        }

        // Switch arm state based on button inputs
        if (gamepad1.dpad_up) {
            currentArmState = ArmState.ARM_UP;
        } else if (gamepad1.dpad_down) {
            currentArmState = ArmState.ARM_DOWN;
        } else if (gamepad1.a){
            currentArmState = ArmState.ARM_SCORE;
        } else if (gamepad1.start) {
            currentArmState = ArmState.ARM_CLIMB;
        } else {
            currentArmState = ArmState.MANUAL_CONTROL;
        }
    }

    private void updateArmPosition() {
        currentPosition = Math.max(Math.min(ARM_UPPER_BOUND, currentPosition), ARM_LOWER_BOUND);

        motorA.setTargetPosition(currentPosition);
        motorB.setTargetPosition(currentPosition);

        motorA.setPower(0.75);
        motorB.setPower(0.75);
        telemetry.addData("CurrentPosition", currentPosition);

    }

    private void pidControlArm() {
        int error = armSetpoint - motorA.getCurrentPosition();
        integral += error;
        double derivative = error - previousError;

        double output = armP * error + armI * integral + armD * derivative;

        currentPosition += output;

        previousError = error;
    }


    //add separate claw code

    private void IntakeMode(){
        if (gamepad1.x) {
            IntakeON();
        } else if (gamepad1.b) {
            IntakeOFF();
        }
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
        armDown();
    }

    private void closeClaw() {
        setClawPosition(0.4);
        armUp();
    }

    private void IntakeON() {
        setClawPosition(0.2);
//        armDown();
        currentPosition = armIntake;
        wrist.setPosition(wristIntake);
    }

    private void IntakeOFF() {

        setClawPosition(0.4);
//        armUp();
        currentPosition = armScorePosition;
        wrist.setPosition(wristOuttake);
    }

    private void setClawPosition(double position) {
        claw1.setPosition(position);
        claw2.setPosition(position);
    }

    private void resetArm() {
        currentPosition = Constants.startingArmPos;
    }
}