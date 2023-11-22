package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.HashMap;
import java.util.Map;

public class Drivetrain {
    Map<DriveMotors, DcMotor> motors = new HashMap<DriveMotors, DcMotor>();
    Gamepad gamepad;
    HardwareMap myHardwarmap;

    YawPitchRollAngles botAngles;

    public Drivetrain(Gamepad _gamepad, HardwareMap _myHardwarmap) {
        gamepad = _gamepad;
        myHardwarmap = _myHardwarmap;
        initMotors();
    }
    public void initMotors() {
        motors.put(DriveMotors.D1, myHardwarmap.dcMotor.get("backRight"));
        motors.put(DriveMotors.D2, myHardwarmap.dcMotor.get("backLeft"));
        motors.put(DriveMotors.D3, myHardwarmap.dcMotor.get("frontRight"));
        motors.put(DriveMotors.D4, myHardwarmap.dcMotor.get("frontLeft"));

        motors.get(DriveMotors.D2).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(DriveMotors.D4).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive() {
        drive((double) gamepad.left_stick_x, -(double) gamepad.left_stick_y, -(double) gamepad.right_stick_x);
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

        motors.get(DriveMotors.D1).setPower(power * cosA/max + turn);
        motors.get(DriveMotors.D2).setPower(power * sinA/max - turn);
        motors.get(DriveMotors.D3).setPower(power * sinA/max + turn);
        motors.get(DriveMotors.D4).setPower(power * cosA/max - turn);

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

        motors.get(DriveMotors.D1).setPower(power * cosA/max + .5);
        motors.get(DriveMotors.D2).setPower(power * sinA/max - .5);
        motors.get(DriveMotors.D3).setPower(power * sinA/max + .5);
        motors.get(DriveMotors.D4).setPower(power * cosA/max - .5);
    }

}
