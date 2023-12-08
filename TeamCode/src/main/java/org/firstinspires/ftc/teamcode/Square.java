package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "square")
public class Square extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(20)
                .turn(90)
                .forward(20)
                .turn(90)
                .forward(20)
                .turn(90)
                .forward(20)
                .turn(90)
                .build();
        waitForStart();
        drive.followTrajectorySequence(t1);
    }
}
