package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.armDropOff;
import static org.firstinspires.ftc.teamcode.Constants.armIntake;
import static org.firstinspires.ftc.teamcode.Constants.controlHubRotation;
import static org.firstinspires.ftc.teamcode.Constants.startingArmPos;
import static org.firstinspires.ftc.teamcode.Constants.wristIntake;
import static org.firstinspires.ftc.teamcode.Constants.wristStowOrOutTake;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name="CompTeleop")
public class Teleop extends OpMode {

    Drivetrain mecanum;
    DcMotorEx motorA, motorB;
    int currentPosition = 100;
    ServoImplEx claw1, claw2, wrist;
    IMU imu;
    int tolerance = 10;
    int armSetpoint = 400;


    @Override
    public void init() {
        mecanum = new Drivetrain(gamepad1, hardwareMap);
        motorA = hardwareMap.get(DcMotorEx.class, "A1");
        motorB = hardwareMap.get(DcMotorEx.class, "A2");
        claw1 = hardwareMap.get(ServoImplEx.class, "leftPixel");
        claw2 = hardwareMap.get(ServoImplEx.class, "rightPixel");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");

        motorB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);

        motorA.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setTargetPositionTolerance(tolerance);
//        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorB.setPower(0);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setTargetPositionTolerance(tolerance);
//        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        wrist.setPosition(.8);

        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            controlHubRotation
        ));
        imu.initialize(parameters);
//        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        myArm = new Arm(hardwareMap, telemetry);
//        while (!gamepad1.x) {
//            telemetry.addData("Encoder PositionA", motorA.getCurrentPosition());
//            telemetry.addData("Encoder PositionB", motorB.getCurrentPosition());
//            telemetry.addData("Encoder Positions C1", claw1.getPosition());
//            telemetry.addData("Encoder Positions C2", claw2.getPosition());
//            telemetry.addData("Encoder Positions W1", wrist.getPosition());
//            telemetry.update();
//        }
        currentPosition = startingArmPos;
    }

    public void loop() {
        mecanum.drive(imu);
        telemetry.addData("YAW",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        currentPosition = Math.min(Math.max(0, currentPosition), 700);

        if (gamepad1.a) {
            mecanum.rotateBy(90, 0.5);
        }

        if (gamepad1.dpad_up) {
            joggArmUp();
        } else if (gamepad1.dpad_down) {
            joggArmDown();
        } else if(gamepad1.x){
            armUp();
        } else if(gamepad1.b) {
            armDown();
        } 

        motorA.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setTargetPositionTolerance(tolerance);
        motorA.setTargetPosition(currentPosition);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorB.setPower(0);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setTargetPositionTolerance(tolerance);
        motorB.setTargetPosition(currentPosition);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorA.setPower(.75);


        motorB.setPower(.75);

//        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double position = motorA.getCurrentPosition();
        double desiredPosition = motorA.getTargetPosition();

        telemetry.addData("Encoder PositionA", position);
        telemetry.addData("Desired PositionA", desiredPosition);

        double positionB = motorB.getCurrentPosition();
        double desiredPositionB = motorB.getTargetPosition();

        telemetry.addData("Encoder PositionB", positionB);
        telemetry.addData("Desired PositionB", desiredPositionB);
        telemetry.addData("Gamepad1", gamepad1.left_trigger);

        if (gamepad1.left_trigger > 0) {
            clawOpen(claw1);
            clawOpen(claw2);
        } else if (gamepad1.right_trigger > 0) {
            closeClaw(claw1);
            closeClaw(claw2);
        }

        telemetry.addData("C1", claw1.getPosition());
        telemetry.addData("C2", claw2.getPosition());

        telemetry.update();
    }


    public void joggArmUp() {
        currentPosition += 50;
    }

    public void joggArmDown() {
        currentPosition -= 50;
    }
    public void armUp() {
        currentPosition = armDropOff;
        wrist.setPosition(wristStowOrOutTake);
    }
    public void armDown() {
        currentPosition = armIntake;
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