package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.roadRunner.util.Encoder;

import java.util.HashMap;
import java.util.Map;

/**
 * The Drivetrain class represents the robot's drivetrain system, including motor control and movement.
 */
public class Drivetrain {

    final double X_FACTOR = .7;
    final double Y_FACTOR = .7;
    final double TURN_FACTOR = .7;
    private Map<DriveMotors, DcMotor> motors = new HashMap<DriveMotors, DcMotor>();
    private Map<EncoderNames, Encoder> encoders = new HashMap<EncoderNames, Encoder>();

    YawPitchRollAngles botAngles;
    private Gamepad gamepad;
    private HardwareMap hardwareMap;

    /**
     * Constructs a Drivetrain instance.
     *
     * @param gamepad     The gamepad for manual control.
     * @param hardwareMap The HardwareMap for accessing motors.
     */
    public Drivetrain(Gamepad gamepad, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
        initMotors();
        initEncoders();
    }

    /**
     * Initializes the motors using the hardware map.
     * Provides basic error handling for motor initialization.
     */
    private void initMotors() {
        try {
            motors.put(DriveMotors.BACK_RIGHT, hardwareMap.dcMotor.get("backRight"));
            motors.put(DriveMotors.BACK_LEFT, hardwareMap.dcMotor.get("backLeft"));
            motors.put(DriveMotors.FRONT_RIGHT, hardwareMap.dcMotor.get("frontRight"));
            motors.put(DriveMotors.FRONT_LEFT, hardwareMap.dcMotor.get("frontLeft"));

            motors.get(DriveMotors.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
            motors.get(DriveMotors.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            // Log or handle the exception accordingly
            e.printStackTrace();
        }
    }

    public void initEncoders() {
        encoders.put(EncoderNames.RIGHT, new Encoder(hardwareMap.get(DcMotorEx.class, "backRight")));
        encoders.put(EncoderNames.LEFT, new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft")));
        encoders.put(EncoderNames.BACK, new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight")));

        encoders.get(EncoderNames.BACK).setDirection(Encoder.Direction.REVERSE);
    }

    /**
     * Drives the robot based on the current gamepad input.
     */
    public void drive() {
        double x = gamepad.left_stick_x * X_FACTOR;
        double y = -gamepad.left_stick_y * Y_FACTOR;
        double turn = -gamepad.right_stick_x * TURN_FACTOR;

        drive(x, y, turn);
    }

    /**
     *
     * @param myIMU Pass in an IMU
     * @brief Field centric driving
     */

    public void drive(IMU myIMU) {
        botAngles = myIMU.getRobotYawPitchRollAngles();

        double botHeading = botAngles.getYaw(AngleUnit.RADIANS);

        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motors.get(DriveMotors.BACK_RIGHT).setPower(backRightPower);
        motors.get(DriveMotors.BACK_LEFT).setPower(backLeftPower);
        motors.get(DriveMotors.FRONT_RIGHT).setPower(frontRightPower);
        motors.get(DriveMotors.FRONT_LEFT).setPower(frontLeftPower);
    }

    /**
     * Drives the robot with specified joystick values.
     *
     * @param x    The x-component of joystick input.
     * @param y    The y-component of joystick input.
     * @param turn The turn (rotation) input.
     */

    public void drive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sinA = Math.sin(theta - Math.PI / 4);
        double cosA = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sinA), Math.abs(cosA));

        // Set power to each motor based on joystick input and turn
        motors.get(DriveMotors.BACK_RIGHT).setPower((power * cosA / max) + turn);
        motors.get(DriveMotors.BACK_LEFT).setPower((power * sinA / max) - turn);
        motors.get(DriveMotors.FRONT_RIGHT).setPower((power * sinA / max) + turn);
        motors.get(DriveMotors.FRONT_LEFT).setPower((power * cosA / max) - turn);

        // If the total power exceeds 1, scale down all motors to prevent overdriving
        if ((power + Math.abs(turn)) > 1) {
            motors.forEach((name, motor) -> motor.setPower(motor.getPower() / (power + turn)));
        }
    }

    /**
     * Rotates the robot by a specified angle with a given power.
     *
     * @param theta The angle in degrees [-180, 180].
     * @param power The power in the range [-1, 1].
     */
    public void rotateBy(double theta, double power) {
        double sinA = Math.sin(Math.toRadians(theta) - Math.PI / 4);
        double cosA = Math.cos(Math.toRadians(theta) - Math.PI / 4);
        double max = Math.max(Math.abs(sinA), Math.abs(cosA));

        // Set power to each motor for rotation
        motors.get(DriveMotors.BACK_RIGHT).setPower((power * cosA / max) + 0.5);
        motors.get(DriveMotors.BACK_LEFT).setPower((power * sinA / max) - 0.5);
        motors.get(DriveMotors.FRONT_RIGHT).setPower((power * sinA / max) + 0.5);
        motors.get(DriveMotors.FRONT_LEFT).setPower((power * cosA / max) - 0.5);
    }

    //test, might not work
    
    public void stop() {
        // Set the power of all motors to zero
        motors.forEach((name, motor) -> motor.setPower(0));
    }
}
