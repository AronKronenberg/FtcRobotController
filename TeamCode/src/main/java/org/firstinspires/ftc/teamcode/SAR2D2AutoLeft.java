package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.INTAKE_WHEEL_SPEED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous Left", preselectTeleOp = "TeleOp")
public class SAR2D2AutoLeft extends LinearOpMode {
    enum State {
        DRIVE_TO_B1,
        SCORE_B1,
        WAIT_1,
        DRIVE_TO_S2,
        DRIVE_F_S2,
        PICK_UP_S2,
        TRANSFER_S2,
        DRIVE_TO_B2,
        SCORE_S2,
        WAIT_2,
        DRIVE_TO_S3,
        DRIVE_F_S3,
        PICK_UP_S3,
        TRANSFER_S3,
        DRIVE_TO_B3,
        SCORE_S3,
        WAIT_3,
        DRIVE_TO_S4,
        PICK_UP_S4,
        TRANSFER_S4,
        DRIVE_TO_B4,
        SCORE_S4,

        IDLE
    }
    State currentState = State.IDLE;
    boolean isTransferring = false;
    ElapsedTime time = new ElapsedTime();

    Hardware robot = new Hardware();

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 24, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        robot.setIntakePitch(0.45);
        robot.closeClaw();

        Trajectory drive_to_b1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5,43, Math.toRadians(-45)))
                .build();

        TrajectorySequence drive_to_s2 = drive.trajectorySequenceBuilder(drive_to_b1.end())
                .splineToLinearHeading(new Pose2d(38.5,22, Math.toRadians(90)), Math.toRadians(0))
                .build();

        TrajectorySequence drive_f_s2 = drive.trajectorySequenceBuilder(drive_to_s2.end())
                .forward(2)
                .build();

        TrajectorySequence drive_to_b2 = drive.trajectorySequenceBuilder(drive_f_s2.end())
                .lineToLinearHeading(new Pose2d(5,43, Math.toRadians(-45)))
                .build();

        TrajectorySequence drive_to_s3 = drive.trajectorySequenceBuilder(drive_to_b2.end())
                .splineToLinearHeading(new Pose2d(38.5, 33, Math.toRadians(90)), Math.toRadians(0))
                .build();

        TrajectorySequence drive_f_s3 = drive.trajectorySequenceBuilder(drive_to_s3.end())
                .forward(2)
                .build();

        TrajectorySequence drive_to_b3 = drive.trajectorySequenceBuilder(drive_f_s3.end())
                .lineToLinearHeading(new Pose2d(5,43, Math.toRadians(-45)))
                .build();

        TrajectorySequence drive_to_s4 = drive.trajectorySequenceBuilder(drive_to_b3.end())
                .splineToLinearHeading(new Pose2d(38.5, 44, Math.toRadians(90)), Math.toRadians(0))
                .build();

        TrajectorySequence drive_f_s4 = drive.trajectorySequenceBuilder(drive_to_s4.end())
                .forward(2)
                .build();

        TrajectorySequence drive_to_b4 = drive.trajectorySequenceBuilder(drive_f_s4.end())
                .lineToLinearHeading(new Pose2d(5,43, Math.toRadians(-45)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.DRIVE_TO_B1;
        robot.intakePitch.setPosition(0.65);
        drive.followTrajectoryAsync(drive_to_b1);
        robot.setOutakeTarget(40);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case DRIVE_TO_B1:
                    if (!drive.isBusy()) {
                        currentState = State.SCORE_B1;
                        robot.setBucketPos(Hardware.BucketState.BUCKET_OUT.getValue());
                        time.reset();
                    }
                    break;
                case SCORE_B1:
                    if (time.milliseconds() >= 1500) {
                        currentState = State.WAIT_1;
                        robot.setBucketPos(Hardware.BucketState.BUCKET_IN.getValue());
                        time.reset();
                    }
                    break;
                case WAIT_1:
                    if (time.milliseconds() >= 500) {
                        currentState = State.DRIVE_TO_S2;
                        robot.setOutakeTarget(0.0);
                        robot.setIntakePitch(0.925);
                        drive.followTrajectorySequenceAsync(drive_to_s2);
                    }
                    break;
                case DRIVE_TO_S2:
                    if (!drive.isBusy() && !robot.isOutakeBusy()) {
                        currentState = State.DRIVE_F_S2;
                        robot.setIntakePitch(1);
                        robot.intakeWheel.setPower(INTAKE_WHEEL_SPEED);
                        drive.followTrajectorySequenceAsync(drive_f_s2);

                        time.reset();
                    }
                    break;
                case DRIVE_F_S2:
                    if (!drive.isBusy()) {
                        currentState = State.PICK_UP_S2;
                        time.reset();
                    }
                    break;
                case PICK_UP_S2:
                    if (time.milliseconds() >= 1500) {
                        robot.intakeWheel.setPower(0);
                        currentState = State.TRANSFER_S2;


                        robot.setIntakePitch(0.4);
                        sleep(1000);
                        robot.intakeWheel.setPower(-1);
                        sleep(500);
                        robot.intakeWheel.setPower(0);
                        robot.setIntakePitch(0.55);
                    }
                    break;
                case TRANSFER_S2:
                    if (!isTransferring) {
                        currentState = State.DRIVE_TO_B2;
                        drive.followTrajectorySequenceAsync(drive_to_b2);
                        robot.setOutakeTarget(40);
                    }
                    break;
                case DRIVE_TO_B2:
                    if (!drive.isBusy() && !robot.isOutakeBusy()) {
                        currentState = State.SCORE_S2;
                        robot.setBucketPos(Hardware.BucketState.BUCKET_OUT.getValue());
                        time.reset();
                    }
                    break;
                case SCORE_S2:
                    if (time.milliseconds() >= 1500) {
                        currentState = State.WAIT_2;
                        robot.setBucketPos(Hardware.BucketState.BUCKET_IN.getValue());
                        time.reset();
                    }
                    break;
                case WAIT_2:
                    if (time.milliseconds() >= 500) {
                        drive.followTrajectorySequenceAsync(drive_to_s3);
                        robot.setOutakeTarget(0);
                        currentState = State.DRIVE_TO_S3;
                        robot.setIntakePitch(0.925);
                    }
                    break;
                case DRIVE_TO_S3:
                    if (!drive.isBusy()) {
                        currentState = State.PICK_UP_S3;
                        robot.setIntakePosition(1);
                        robot.intakeWheel.setPower(INTAKE_WHEEL_SPEED);
                        drive.followTrajectorySequenceAsync(drive_f_s3);
                        time.reset();
                    }
                    break;
                case DRIVE_F_S3:
                    if (!drive.isBusy()) {
                        currentState = State.PICK_UP_S3;
                        time.reset();
                    }
                    break;
                case PICK_UP_S3:
                    if (time.milliseconds() >= 1500) {
                        currentState = State.TRANSFER_S3;


                        robot.setIntakePitch(0.4);
                        sleep(1000);
                        robot.intakeWheel.setPower(-1);
                        sleep(500);
                        robot.intakeWheel.setPower(0);
                        robot.setIntakePitch(0.55);
                    }
                    break;
                case TRANSFER_S3:
                    if (!isTransferring) {
                        currentState = State.DRIVE_TO_B3;
                        drive.followTrajectorySequenceAsync(drive_to_b3);
                        robot.setOutakeTarget(40);
                    }
                    break;
                case DRIVE_TO_B3:
                    if (!drive.isBusy()) {
                        currentState = State.SCORE_S3;
                        robot.setBucketPos(Hardware.BucketState.BUCKET_OUT.getValue());
                        time.reset();
                    }
                    break;
                case SCORE_S3:
                    if (time.milliseconds() >= 1000) {
                        robot.setBucketPos(Hardware.BucketState.BUCKET_IN.getValue());
                        currentState = State.WAIT_3;
                    }
                    break;
                case WAIT_3:
                    if (time.milliseconds() >= 500) {
                        drive.followTrajectorySequenceAsync(drive_to_s4);
                        robot.setOutakeTarget(0);
                        currentState = State.DRIVE_TO_S4;
                        robot.setIntakePitch(0.925);
                    }
                    break;
                case DRIVE_TO_S4:
                    if (!drive.isBusy()) {
                        currentState = State.PICK_UP_S4;
                        robot.setIntakePitch(1);
                        robot.intakeWheel.setPower(INTAKE_WHEEL_SPEED);
                        drive.followTrajectorySequenceAsync(drive_f_s4);
                        time.reset();
                    }
                    break;
                case PICK_UP_S4:
                    if (time.milliseconds() >= 1500) {
                        currentState = State.TRANSFER_S4;

                        robot.setIntakePitch(0.4);
                        sleep(1000);
                        robot.intakeWheel.setPower(-1);
                        sleep(500);
                        robot.intakeWheel.setPower(0);
                        robot.setIntakePitch(0.55);
                    }
                    break;
                case TRANSFER_S4:
                    if (!isTransferring) {
                        currentState = State.DRIVE_TO_B4;
                        drive.followTrajectorySequenceAsync(drive_to_b4);
                        robot.setOutakeTarget(40);
                    }
                    break;
                case DRIVE_TO_B4:
                    if (!drive.isBusy()) {
                        currentState = State.SCORE_S4;
                        robot.setBucketPos(Hardware.BucketState.BUCKET_OUT.getValue());
                        time.reset();
                    }
                    break;
                case SCORE_S4:
                    if (time.milliseconds() >= 1000) {
                        robot.setBucketPos(Hardware.BucketState.BUCKET_IN.getValue());
                        currentState = State.IDLE;
                    }
                case IDLE:
                    break;
            }

            drive.update();
            double error = 0;
            robot.updateOutake();

            if (isTransferring){
                updateTransfer();
            }

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

    public void updateTransfer() {

        telemetry.addData("Status", "Transferring...");
        telemetry.update();

        if (time.milliseconds() <= 500) {
            robot.setIntakePitch(0.4);
        }
        else if (time.milliseconds() <= 1000) {
            robot.intakeWheel.setPower(-1);
        }
        else if (time.milliseconds() <= 1250) {
            robot.intakeWheel.setPower(0);
            robot.setIntakePitch(0.55);
        }
        else if (time.milliseconds() <= 1500) {
            robot.setOutakeTarget(5.0);
            robot.updateOutake();
            robot.setBucketPos(0.4);
        }
        else {
            isTransferring = false;
        }
    }
}
