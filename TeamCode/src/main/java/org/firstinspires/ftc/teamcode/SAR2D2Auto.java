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
        DRIVE_TO_BAR_FSP,
        RETRACT_OUTAKE_FSP,
        OPEN_CLAW_FSP,
        DRIVE_TO_FSA,
        PUSH_FSA,
        DRIVE_TO_SSA,
        PICK_UP_SSP,
        CLOSE_CLAW_SSP,
        WAIT_SSP,
        RAISE_OUTAKE_SSP,
        PLACE_SSP,
        RETRACT_OUTAKE_SSP,
        OPEN_CLAW_SSP,
        PICK_UP_TSP,
        WAIT_TSP,
        CLOSE_CLAW_TSP,
        PLACE_TSP,
        RETRACT_OUTAKE_TSP,
        OPEN_CLAW_TSP,
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

        robot.setIntakePitch(0.45);
        robot.closeClaw();

        Trajectory drive_to_bar_fsp = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(29.25, 0, Math.toRadians(-90)))
                .build();

        TrajectorySequence drive_to_fsa = drive.trajectorySequenceBuilder(drive_to_bar_fsp.end())
                .strafeRight(1.5)
                .lineToLinearHeading(new Pose2d(25, -25.5, Math.toRadians(90)))
                .build();

        TrajectorySequence push_fsa = drive.trajectorySequenceBuilder(drive_to_fsa.end())
                .strafeRight(31)
                .back(11)
                .strafeLeft(48)
                .build();

        TrajectorySequence drive_to_ssa = drive.trajectorySequenceBuilder(push_fsa.end())
                .lineToLinearHeading(push_fsa.end().plus(new Pose2d(38, 0, Math.toRadians(180))))
                .strafeRight(6)
                .back(10.25)
                .strafeLeft(44)
                .strafeRight(5)
                .forward(16.5)
                .build();

        TrajectorySequence pick_up_ssp = drive.trajectorySequenceBuilder(push_fsa.end())
                .strafeRight(4)
                .forward(10.25)
                .lineToLinearHeading(new Pose2d(2.15, -28.5, Math.toRadians(90)))
                .build();

        TrajectorySequence place_ssp = drive.trajectorySequenceBuilder(pick_up_ssp.end())
                .strafeRight(10)
                .lineToLinearHeading(new Pose2d(24.25, 4, Math.toRadians(-90)))
                .strafeLeft(3)
                .build();

        TrajectorySequence pick_up_tsp = drive.trajectorySequenceBuilder(place_ssp.end())
                .lineToLinearHeading(new Pose2d(2.15, -28.5, Math.toRadians(90)))
                .build();

        TrajectorySequence place_tsp = drive.trajectorySequenceBuilder(pick_up_tsp.end())
                .strafeRight(3)
                .lineToLinearHeading(new Pose2d(24.25, 8, Math.toRadians(-90)))
                .strafeLeft(3)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        currentState = State.DRIVE_TO_BAR_FSP;

        robot.setBucketPos(Hardware.BucketState.BUCKET_OUT.getValue());
        robot.intakePitch.setPosition(0.65);
        drive.followTrajectoryAsync(drive_to_bar_fsp);
        robot.setOutakeTarget(18);

        boolean manualOutake = false;

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case DRIVE_TO_BAR_FSP:
                    if (!drive.isBusy() && !robot.isOutakeBusy()) {
                        currentState = State.RETRACT_OUTAKE_FSP;
                        manualOutake = true;
                        robot.outakeMotor.setPower(-0.5);
                        time.reset();
                    }
                    break;
                case RETRACT_OUTAKE_FSP:
                    if (time.milliseconds() >= 500) {
                        robot.outakeMotor.setPower(0);
                        manualOutake = false;
                        robot.setOutakeTarget(robot.outakeMotor.getCurrentPosition() / DriveConstants.OUTAKE_COUNTS_PER_INCH);
                        currentState = State.OPEN_CLAW_FSP;
                        robot.openClaw();
                        time.reset();
                    }
                    break;
                case OPEN_CLAW_FSP:
                    if (time.milliseconds() >= 500) {
                        currentState = State.DRIVE_TO_FSA;
                        drive.followTrajectorySequenceAsync(drive_to_fsa);
                        robot.setOutakeTarget(-0.7);
                    }
                    break;
                case DRIVE_TO_FSA:
                    if (!drive.isBusy()) {
                        currentState = State.PUSH_FSA;
                        drive.followTrajectorySequenceAsync(push_fsa);
                    }
                    break;
                case PUSH_FSA:
                    if (!drive.isBusy()) {
                        currentState = State.PICK_UP_SSP;
                        drive.followTrajectorySequenceAsync(pick_up_ssp);
                        //drive.followTrajectorySequenceAsync(drive_to_ssa);
                    }
                    break;
//                case DRIVE_TO_SSA:
//                    if (!drive.isBusy()) {
//                        currentState = State.PICK_UP_SSP;
//                        drive.followTrajectoryAsync(pick_up_ssp);
//                    }
//                    break;
                case PICK_UP_SSP:
                    if (!drive.isBusy()) {
                        currentState = State.CLOSE_CLAW_SSP;
                        robot.closeClaw();
                        time.reset();
                    }
                    break;
                case CLOSE_CLAW_SSP:
                    if (time.milliseconds() >= 500) {
                        currentState = State.WAIT_SSP;
                        robot.setOutakeTarget(23);
                        time.reset();
                    }
                    break;
                case WAIT_SSP:
                    if (time.milliseconds() >= 1500) {
                        currentState = State.RAISE_OUTAKE_SSP;
                    }
                    break;
                case RAISE_OUTAKE_SSP:
                    if (!robot.isOutakeBusy()) {
                        currentState = State.PLACE_SSP;
                        drive.followTrajectorySequenceAsync(place_ssp);
                    }
                    break;
                case PLACE_SSP:
                    if (!drive.isBusy()) {
                        currentState = State.RETRACT_OUTAKE_SSP;
                        manualOutake = true;
                        robot.outakeMotor.setPower(-0.5);
                        time.reset();
                    }
                    break;
                case RETRACT_OUTAKE_SSP:
                    if (time.milliseconds() >= 500) {
                        manualOutake = false;
                        robot.outakeMotor.setPower(0);
                        robot.setOutakeTarget(robot.outakeMotor.getCurrentPosition() / DriveConstants.OUTAKE_COUNTS_PER_INCH);
                        currentState = State.OPEN_CLAW_SSP;
                        robot.openClaw();
                        time.reset();
                    }
                    break;
                case OPEN_CLAW_SSP:
                    if (time.milliseconds() >= 250) {
                        currentState = State.PICK_UP_TSP;
                        drive.followTrajectorySequenceAsync(pick_up_tsp);
                        robot.setOutakeTarget(-0.7);
                    }
                    break;
                case PICK_UP_TSP:
                    if (!drive.isBusy()) {
                        currentState = State.CLOSE_CLAW_TSP;
                        robot.closeClaw();
                        time.reset();
                    }
                    break;
                case CLOSE_CLAW_TSP:
                    if (time.milliseconds() > 500) {
                        robot.setOutakeTarget(19);
                        currentState = State.WAIT_TSP;
                        time.reset();
                    }
                case WAIT_TSP:
                    if (time.milliseconds() >= 1000) {
                        currentState = State.PLACE_TSP;
                        drive.followTrajectorySequenceAsync(place_tsp);
                    }
                    break;
                case PLACE_TSP:
                    if (!drive.isBusy() && !robot.isOutakeBusy()) {
                        currentState = State.RETRACT_OUTAKE_TSP;
                        manualOutake = true;
                        robot.outakeMotor.setPower(-1);
                        time.reset();
                    }
                    break;
                case RETRACT_OUTAKE_TSP:
                    if (time.milliseconds() >= 250) {
                        currentState = State.OPEN_CLAW_TSP;
                        manualOutake = false;
                        robot.outakeMotor.setPower(0);
                        robot.setOutakeTarget(robot.outakeMotor.getCurrentPosition() / DriveConstants.OUTAKE_COUNTS_PER_INCH);
                        robot.openClaw();
                        time.reset();
                    }
                    break;
                case OPEN_CLAW_TSP:
                    if (time.milliseconds() >= 250) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
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
