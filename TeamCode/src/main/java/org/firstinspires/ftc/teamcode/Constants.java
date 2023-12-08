package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class Constants {
    public static final int tolerance = 10;
    public static final int armStow = 400;
    public static final int startingArmPos = 100;
    public static final int armDropOff = 564;
    public static final int armIntake = 1;
    public static final double wristStowOrOutTake = .9;
    public static final double wristIntake = .2;

    //Quartenion for ControlHUB
    public static RevHubOrientationOnRobot revHubOrientation= new RevHubOrientationOnRobot(
            // Check Values
            new Quaternion(
                    (float) Math.cos(.5 * 90),
                    (float) Math.sin(.5 * 90),
                    0.0f,
                    0.0f,
                    System.nanoTime()
            )
    );
}
