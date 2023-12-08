package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import org.firstinspires.ftc.teamcode.roadRunner.util.Encoder;

import java.util.HashMap;
import java.util.Map;

public class Drivetrain {
    Map<DriveMotors, DcMotor> motors = new HashMap<DriveMotors, DcMotor>();
    Map<EncoderNames, Encoder> encoders = new HashMap<EncoderNames, Encoder>();
    Gamepad gamepad;
    HardwareMap myHardwarmap;

    YawPitchRollAngles botAngles;

    public Drivetrain(Gamepad _gamepad, HardwareMap _myHardwarmap) {
        gamepad = _gamepad;
        myHardwarmap = _myHardwarmap;
        initMotors();
        initEncoders();
    }
    public void initMotors() {
        motors.put(DriveMotors.BACK_RIGHT, myHardwarmap.dcMotor.get("backRight`"));
        motors.put(DriveMotors.BACK_LEFT, myHardwarmap.dcMotor.get("backLeft"));
        motors.put(DriveMotors.FRONT_RIGHT, myHardwarmap.dcMotor.get("frontRight"));
        motors.put(DriveMotors.FRONT_LEFT, myHardwarmap.dcMotor.get("frontLeft"));

        motors.get(DriveMotors.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(DriveMotors.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initEncoders() {
        encoders.put(EncoderNames.LEFT, myHardwarmap.get(Encoder.class, "Left Encoder"));
        encoders.put(EncoderNames.RIGHT, myHardwarmap.get(Encoder.class, "Right Encoder"));
        encoders.put(EncoderNames.BACK, myHardwarmap.get(Encoder.class, "Back Encoder"));

        encoders.get(EncoderNames.BACK).setDirection(Encoder.Direction.REVERSE);
    }

    public void drive() {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double turn = -gamepad.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        //double D1power = Range.clip(x + y + turn)

        double sinA = Math.sin(theta - Math.PI / 4);
        double cosA = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sinA), Math.abs(cosA));

        motors.get(DriveMotors.BACK_RIGHT).setPower(power * cosA/max + turn);
        motors.get(DriveMotors.BACK_LEFT).setPower(power * sinA/max - turn);
        motors.get(DriveMotors.FRONT_RIGHT).setPower(power * sinA/max + turn);
        motors.get(DriveMotors.FRONT_LEFT).setPower(power * cosA/max - turn);

        if ((power + Math.abs(turn)) > 1) {
            motors.forEach((name, motor) -> {
                motor.setPower(motor.getPower() / (power + turn));
            });
        }
    }

    /**
     *
     * @param myIMU Pass in an IMU
     * @brief Field centric driving
     */

    public void drive(IMU myIMU) {
        myIMU.resetYaw();

        botAngles = myIMU.getRobotYawPitchRollAngles();

        double botYaw = botAngles.getYaw(AngleUnit.RADIANS);

        double rotX = gamepad.left_stick_x * Math.cos(-botYaw) - -gamepad.left_stick_y * Math.sin(-botYaw);
        double rotY = gamepad.left_stick_x * Math.sin(-botYaw) + -gamepad.left_stick_y * Math.cos(-botYaw);

        drive(rotX, rotY, -gamepad.right_stick_x);
    }

    /**
     *
     * @param x
     * @param y
     * @param turn
     */

    public void drive(double x, double y, double turn) {

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sinA = Math.sin(theta - Math.PI / 4);
        double cosA = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sinA), Math.abs(cosA));

        motors.get(DriveMotors.BACK_RIGHT).setPower(power * cosA/max + turn);
        motors.get(DriveMotors.BACK_LEFT).setPower(power * sinA/max - turn);
        motors.get(DriveMotors.FRONT_RIGHT).setPower(power * sinA/max + turn);
        motors.get(DriveMotors.FRONT_LEFT).setPower(power * cosA/max - turn);

        if ((power + Math.abs(turn)) > 1) {
            motors.forEach((name, motor) -> {
                motor.setPower(motor.getPower() / (power + turn));
            });
        }
    }

    /**
     *
     * @param theta in degrees [-180, 180]
     * @param power in range [-1, 1]
     */
    public void rotateBy(double theta, double power){
        double sinA = Math.sin(theta - Math.PI / 4);
        double cosA = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sinA), Math.abs(cosA));

        motors.get(DriveMotors.BACK_RIGHT).setPower(power * cosA/max + .5);
        motors.get(DriveMotors.BACK_LEFT).setPower(power * sinA/max - .5);
        motors.get(DriveMotors.FRONT_RIGHT).setPower(power * sinA/max + .5);
        motors.get(DriveMotors.FRONT_LEFT).setPower(power * cosA/max - .5);
    }

}
