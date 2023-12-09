package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ArmClawMovement {

    DcMotorEx motorA, motorB;
    private ServoImplEx claw1, claw2, wrist;

    private final int tolerance = 10;

    public void init(HardwareMap hardwareMap) {
        motorA = hardwareMap.get(DcMotorEx.class, "A1");
        motorB = hardwareMap.get(DcMotorEx.class, "A2");
        claw1 = hardwareMap.get(ServoImplEx.class, "C1");
        claw2 = hardwareMap.get(ServoImplEx.class, "C2");
        wrist = hardwareMap.get(ServoImplEx.class, "W1");

        motorB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor initialization
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        updateArmPosition(startingArmPos);
    }

    public void updateArmPosition(int targetPosition) {
        motorA.setTargetPosition(targetPosition);
        motorB.setTargetPosition(targetPosition);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorA.setPower(.75);
        motorB.setPower(.75);
    }

    public void updateClaw(boolean open) {
        double pos = open ? 0.2 : 0.4;
        claw1.setPosition(pos);
        claw2.setPosition(pos);
    }

    public void armUp(int currentPosition) {
        currentPosition = armDropOff;
    }

    public void armDown(int currentPosition) {
        currentPosition = armIntake;
    }

    public void joggArmUp(int currentPosition) {
        currentPosition += 50;
        wrist.setPosition(wristStowOrOutTake);
    }

    public void joggArmDown(int currentPosition) {
        currentPosition -= 50;
        wrist.setPosition(wristIntake);
    }

    public void closeClaw(Servo claw) {
        claw.setPosition(.4);
    }
    public void clawOpen(Servo claw) {
        claw.setPosition(.2);
    }
    public void setClaw(Servo claw, double units) {
        claw.setPosition(units);
    }
}