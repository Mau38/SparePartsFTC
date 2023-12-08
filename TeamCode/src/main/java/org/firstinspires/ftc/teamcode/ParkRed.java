package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "ParkRed", group = "CompAutos")
public class ParkRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(3)
                .turn((-Math.PI / 2.0) * 1.5)
                .forward(40)
                .build();

        waitForStart();

        drive.followTrajectorySequence(t1);
    }
}
