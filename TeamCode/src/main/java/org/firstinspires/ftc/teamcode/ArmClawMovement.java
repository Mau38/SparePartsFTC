package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ArmClawMovement {

    private ServoImplEx claw1, claw2, wrist;

    private double curretPosition;

    private DcMotorEx motorB, motorA;

    public void init(HardwareMap hardwareMap) {
        claw1 = hardwareMap.get(ServoImplEx.class, "leftPixel");
        claw2 = hardwareMap.get(ServoImplEx.class, "rightPixel");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");

        motorB = hardwareMap.get(DcMotorEx.class, "A2");
        motorA = hardwareMap.get(DcMotorEx.class, "A1");

        motorB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor initialization
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        updateArmPosition(startingArmPos);
        updateClaw(false);
        curretPosition = 0;
    }

    public void loop(){
        //handleClawControls(new Gamepad());
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

//    public void armUp(int currentPosition) {
//        currentPosition = armDropOff;
//    }
//
//    public void armDown(int currentPosition) {
//        currentPosition = armIntake;
//    }
//
//    public void joggArmUp(int currentPosition) {
//        currentPosition += 50;
//        wrist.setPosition(wristStowOrOutTake);
//    }
//
//    public void joggArmDown(int currentPosition) {
//        currentPosition -= 50;
//        wrist.setPosition(wristIntake);
//    }

    public void closeClaw(Servo claw) {
        claw.setPosition(.4);
    }
    public void clawOpen(Servo claw) {
        claw.setPosition(.2);
    }
    public void setClaw(Servo claw, double units) {
        claw.setPosition(units);
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

    public void handleClawControls(Gamepad gamepad1) {
        if(gamepad1.left_trigger > 0){
            openClaw();
            curretPosition = 0.4;
        }
        else if (gamepad1.right_trigger > 0){
            closeClaw();
            curretPosition = 0.2;
        }
        else{
            claw1.setPosition(curretPosition);
            claw2.setPosition(curretPosition);
        }
    }
}