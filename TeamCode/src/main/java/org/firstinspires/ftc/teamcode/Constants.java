package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

@Config
public class Constants {
    public static final int armStow = 400;
    public static final int startingArmPos = -50;
    public static final int armDropOff = 564;
    public static int armScorePosition = -730;
    public static int armClimbPosition = -460;
    public static int armIntake = -50;
    public static int ARM_UPPER_BOUND = -50;
    public static int ARM_LOWER_BOUND = -800;
    public static final double wristStowOrOutTake = .9;
    public static final double wristIntake = .2;
//    public static int armTeleopStartPosition = -50;

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
