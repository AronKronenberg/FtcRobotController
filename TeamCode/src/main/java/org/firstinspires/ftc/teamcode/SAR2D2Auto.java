package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous", preselectTeleOp = "TeleOp")
public class SAR2D2Auto extends LinearOpMode {
    enum State {
        DRIVE_TO_BAR_FS,
        RETRACT_OUTAKE_FS,
        OPEN_CLAW_FS,
        IDLE
    }
    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware();
        robot.init(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        robot.setIntakePitch(DriveConstants.INTAKE_INIT_PITCH);
        robot.closeClaw();

        Trajectory drive_to_bar_fs = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30.5, -6, Math.toRadians(-90)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.DRIVE_TO_BAR_FS;

        drive.followTrajectoryAsync(drive_to_bar_fs);
        robot.setOutakeTarget(19);

        boolean manualOutake = false;

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case DRIVE_TO_BAR_FS:
                    if (!drive.isBusy() && !robot.isOutakeBusy()) {
                        currentState = State.RETRACT_OUTAKE_FS;
                        manualOutake = true;
                        robot.outakeMotor.setPower(-0.5);
                        time.reset();
                    }
                    break;
                case RETRACT_OUTAKE_FS:
                    if (time.milliseconds() >= 500) {
                        robot.outakeMotor.setPower(0);
                        manualOutake = false;
                        currentState = State.OPEN_CLAW_FS;
                        robot.openClaw();
                        time.reset();
                    }
                    break;
                case OPEN_CLAW_FS:
                    if (time.milliseconds() >= 500) {
                        currentState = State.IDLE;
                    }
            }

            drive.update();
            double error = 0;
            if (!manualOutake) error = robot.updateOutake();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addLine();

            telemetry.addData("Current State", currentState);

            String outakeBusy = robot.isOutakeBusy() ? "is busy" : "is not busy";
            telemetry.addData("Outake", outakeBusy);
            telemetry.addData("Outake error", error);

            telemetry.update();
        }
    }
}
