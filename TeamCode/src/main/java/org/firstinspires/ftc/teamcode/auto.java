package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "auto")
public class auto extends LinearOpMode {

    private Drivetrain mecanum;
    private DcMotorEx motorA, motorB;
    private Servo claw1, claw2, wrist;

    private final int tolerance = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanum = new Drivetrain(gamepad1, hardwareMap);
        motorA = hardwareMap.get(DcMotorEx.class, "A1");
        motorB = hardwareMap.get(DcMotorEx.class, "A2");
        claw1 = hardwareMap.get(Servo.class, "C1");
        claw2 = hardwareMap.get(Servo.class, "C2");
        wrist = hardwareMap.get(Servo.class, "W1");

        motorB.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime et = new ElapsedTime();

        initializeMotors();
        waitForStart();

        //change to 100 and 3 to make it go forward for 3 seconds at 100% motor capacity

        // 1st Auto: Go forward for 3 seconds at 50% motor capacity MAIN AUTO!!!!!!!!!
        moveForwardForDuration(0.5, 3);

        // Stop all motors
        stopMotors();
    }

    private void initializeMotors() {
        // Initialize motorA
        motorA.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setTargetPositionTolerance(tolerance);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setTargetPosition(100); // may need to change target position
        motorA.setPower(0.5);

        // Initialize motorB
        motorB.setPower(0);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setTargetPositionTolerance(tolerance);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setTargetPosition(100); // may need to change target position
        motorB.setPower(0.5);
    }

    private void moveForwardForDuration(double power, double seconds) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < seconds) {
            mecanum.drive(power, power, 0);
        }
    }

    private void stopMotors() {
        mecanum.stop();

        // Stop motorA
        motorA.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop motorB
        motorB.setPower(0);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
