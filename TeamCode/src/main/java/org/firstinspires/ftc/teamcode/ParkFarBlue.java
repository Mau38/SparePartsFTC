package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "ParkFarBlue", group = "CompAutos")
public class ParkFarBlue extends LinearOpMode {

    public static double STRAFE_DIST = 30;
    public static double FORWARD_DIST = 160;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d()).strafeRight(STRAFE_DIST).forward(FORWARD_DIST).build();

        waitForStart();

        drive.followTrajectorySequence(t1);
    }
}
