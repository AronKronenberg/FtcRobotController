package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonomousAsync extends LinearOpMode {
    enum State {
        DRIVE_TO_BAR, // lift outake while doing this
        RETRACT_OUTAKE,
        OPEN_CLAW,
        DRIVE_TO_PIECE_ONE,
        PUSH_PIECE_ONE,
        DRIVE_TO_PIECE_TWO,
        PUSH_PIECE_TWO,
        DRIVE_TO_SPECIMEN_ONE,
        CLOSE_CLAW,
        LIFT_OUTAKE_AWAY,
        DRIVE_TO_BAR_SS, // lift outake while doing this
        RETRACT_OUTAKE_SS,
        OPEN_CLAW_SS,
        DRIVE_TO_SPECIMEN_TWO, // fully retract
        CLOSE_CLAW_TS,
        LIFT_OUTAKE_TS,
        DRIVE_TO_BAR_TS, // lift outake while doing this
        RETRACT_OUTAKE_TS,
        OPEN_CLAW_TS,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware();
        robot.init(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        robot.setIntakePitch(DriveConstants.INTAKE_INIT_PITCH);
        robot.closeClaw();

        Trajectory drive_to_bar = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-6, 30.5,  0))
                .build();

        // outake down, claw open

        // outake down almost fully
        TrajectorySequence drive_to_piece_one = drive.trajectorySequenceBuilder(drive_to_bar.end())
                .lineToLinearHeading(new Pose2d(24, 30.5, 0))
                .lineToLinearHeading(new Pose2d(31, 36, 0))
                .build();

        Trajectory push_piece_one = drive.trajectoryBuilder(drive_to_piece_one.end())
                .strafeRight(12)
                .build();


        Trajectory drive_to_piece_two = drive.trajectoryBuilder(push_piece_one.end())
                .lineToLinearHeading(new Pose2d(41, 24, 0))
                .build();

        Trajectory push_piece_two = drive.trajectoryBuilder(drive_to_piece_two.end())
                .strafeRight(12)
                .build();

        // lower outake while
        Trajectory drive_to_specimen_one = drive.trajectoryBuilder(push_piece_two.end())
                .lineToLinearHeading(new Pose2d(33.5, 3, Math.toRadians(180)))
                .build();

        //close claw, raise outtake
        Trajectory drive_to_bar_ss = drive.trajectoryBuilder(drive_to_specimen_one.end())
                .lineToLinearHeading(new Pose2d(-3, 30.5, Math.toRadians(0)))
                .build();

        //lower outtake, open claw

        Trajectory drive_to_specimen_two = drive.trajectoryBuilder(drive_to_bar_ss.end())
                .lineToLinearHeading(new Pose2d(33.5, 3, Math.toRadians(180)))
                .build();

        //close claw, raise outtake

        Trajectory drive_to_bar_ts = drive.trajectoryBuilder(drive_to_specimen_two.end())
                .lineToLinearHeading(new Pose2d(-3, 30.5, Math.toRadians(0)))
                .build();

        // lower outake, open claw

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.DRIVE_TO_BAR;
        drive.followTrajectoryAsync(drive_to_bar);
        robot.outakeTarget = 19.0;

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            // our state machine logic
            switch (currentState) {
                case DRIVE_TO_BAR:
                    if (!drive.isBusy()) {
                        currentState = State.RETRACT_OUTAKE;
//                        robot.outakeMotor.setPower(-0.5);
//                        sleep(500);
                    }
                    break;
                case RETRACT_OUTAKE:
                    currentState = State.OPEN_CLAW;
                    robot.openClaw();
                    time.reset();

                    break;
                case OPEN_CLAW:
                    if (time.milliseconds() >= 500) {
                        currentState = State.DRIVE_TO_PIECE_ONE;
                        drive.followTrajectorySequenceAsync(drive_to_piece_one);
                    }
                    break;
                case DRIVE_TO_PIECE_ONE:
                    if (!drive.isBusy()) {
                        currentState = State.PUSH_PIECE_ONE;
                        drive.followTrajectoryAsync(push_piece_one);
                    }
                    break;
                case PUSH_PIECE_ONE:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_PIECE_TWO;
                        drive.followTrajectoryAsync(drive_to_piece_two);
                    }
                    break;
                case DRIVE_TO_PIECE_TWO:
                    if (!drive.isBusy()) {
                        currentState = State.PUSH_PIECE_TWO;
                        drive.followTrajectoryAsync(push_piece_two);
                    }
                    break;
                case PUSH_PIECE_TWO:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_SPECIMEN_ONE;
                        drive.followTrajectoryAsync(drive_to_specimen_one);
                    }
                    break;
                case DRIVE_TO_SPECIMEN_ONE:
                    if (!drive.isBusy()) {
                        currentState = State.CLOSE_CLAW;
                        robot.closeClaw();
                        time.reset();
                    }
                    break;
                case CLOSE_CLAW:
                    if (time.milliseconds() >= 500) {
                        currentState = State.LIFT_OUTAKE_AWAY;
                        robot.outakeTarget = 19.0;
                    }
                    break;
                case LIFT_OUTAKE_AWAY:
                    if (!robot.outakeMotor.isBusy()) {
                        currentState = State.DRIVE_TO_BAR_SS;
                        drive.followTrajectoryAsync(drive_to_bar_ss);
                    }
                    break;
                case DRIVE_TO_BAR_SS:
                    if (!drive.isBusy()) {
                        currentState = State.RETRACT_OUTAKE_SS;
                        robot.outakeMotor.setPower(-0.5);
                        sleep(500);
                    }
                    break;
                case RETRACT_OUTAKE_SS:
                    currentState = State.OPEN_CLAW_SS;
                    robot.openClaw();
                    time.reset();

                    break;
                case OPEN_CLAW_SS:
                    if (time.milliseconds() >= 500) {
                        currentState = State.DRIVE_TO_SPECIMEN_TWO;
                        drive.followTrajectoryAsync(drive_to_specimen_two);
                    }
                    break;
                case DRIVE_TO_SPECIMEN_TWO:
                    if (!drive.isBusy()) {
                        currentState = State.CLOSE_CLAW_TS;
                        robot.closeClaw();
                        time.reset();
                    }
                    break;
                case CLOSE_CLAW_TS:
                    if (time.milliseconds() >= 500) {
                        currentState = State.LIFT_OUTAKE_TS;
                        robot.outakeTarget = 19;
                    }
                    break;
                case LIFT_OUTAKE_TS:
                    if (!robot.outakeMotor.isBusy()) {
                        currentState = State.DRIVE_TO_BAR_TS;
                        drive.followTrajectoryAsync(drive_to_bar_ts);
                    }
                    break;
                case DRIVE_TO_BAR_TS:
                    if (!drive.isBusy()) {
                        currentState = State.RETRACT_OUTAKE_TS;
                        robot.outakeMotor.setPower(-0.5);
                        sleep(500);
                    }
                    break;
                case OPEN_CLAW_TS:
                    robot.openClaw();
                    break;
                case IDLE:
                    break;
            }
            drive.update();
            robot.updateOutake();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Current State", currentState);
            telemetry.update();
        }
    }
}
