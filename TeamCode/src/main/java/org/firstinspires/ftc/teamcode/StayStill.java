package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class StayStill extends LinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Trajectory forward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .forward(1e-6)
                    .build();
            drive.followTrajectory(forward);
            Trajectory backward = drive.trajectoryBuilder(forward.end())
                    .back(1e-6)
                    .build();
            drive.followTrajectory(backward);
        }
    }
}
