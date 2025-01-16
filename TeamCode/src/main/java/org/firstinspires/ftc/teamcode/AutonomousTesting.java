package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="Autonomous Testing", preselectTeleOp = "TeleOp")
public class AutonomousTesting extends LinearOpMode {
    Hardware robot = new Hardware();
    SampleMecanumDrive drive;

    public static double intakePos = 0;
    public static double outakePos = 0;

    public enum AllianceSide {
        RIGHT,
        LEFT
    }

    private AllianceSide side = AllianceSide.RIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        drive = new SampleMecanumDrive(hardwareMap);

        robot.outakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.outakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.moveOutake(5.0);
        robot.moveOutake(-5.0);
        robot.moveIntake(5.0);
        robot.moveIntake(-5.0);

        telemetry.addData("Press Dpad Left for LEFT", "<");
        telemetry.addData("Press Dpad Right for RIGHT", ">");
        telemetry.addData("Current Side", side);
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                side = AllianceSide.LEFT;
            }
            else if (gamepad1.dpad_right) {
                side = AllianceSide.RIGHT;
            }

            telemetry.addData("Press Dpad Left for LEFT", "<");
            telemetry.addData("Press Dpad Right for RIGHT", ">");
            telemetry.addData("Current Side", side);
            telemetry.update();
        }


        robot.setIntakePitch(DriveConstants.INTAKE_INIT_PITCH);
        robot.closeClaw();

        waitForStart();

        Pose2d initPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(initPose);

        switch (side) {
            case RIGHT:
                Trajectory moveToBar = drive.trajectoryBuilder(initPose)
                        .lineToLinearHeading(new Pose2d(-6, 27,  0))
                        .build();
                drive.followTrajectory(moveToBar);

                // move outake up to the bar
                robot.moveOutake(19.0);

                // drive into the bar
                Trajectory moveIntoBar = drive.trajectoryBuilder(moveToBar.end())
                        .lineToLinearHeading(new Pose2d(moveToBar.end().getX(), moveToBar.end().getY() + 3.5, moveToBar.end().getHeading()))
                        .build();
                drive.followTrajectory(moveIntoBar);

                //robot.moveOutake(-3.0);
                robot.outakeMotor.setPower(-0.5);
                sleep(500);
                robot.outakeMotor.setPower(robot.outakeController.getF());
                robot.openClaw();

                Trajectory moveAwayFromBar = drive.trajectoryBuilder(moveIntoBar.end())
                        .lineToLinearHeading(new Pose2d(moveIntoBar.end().getX(), moveIntoBar.end().getY() - 9, moveIntoBar.end().getHeading()))
                        .build();
                drive.followTrajectory(moveAwayFromBar);

                Trajectory moveTowardsSamples = drive.trajectoryBuilder(moveAwayFromBar.end())
                        .lineToLinearHeading(new Pose2d(moveAwayFromBar.end().getX() + 33, moveAwayFromBar.end().getY(), Math.toRadians(90)))
                        .build();
                drive.followTrajectory(moveTowardsSamples);

                robot.setIntakePosition(17.5);

                //sleep(750);

                // push a block
                TrajectorySequence pushAndReset1 = drive.trajectorySequenceBuilder(moveTowardsSamples.end())
                        .turn(Math.toRadians(-110))
                        .turn(Math.toRadians(110))
                        .build();
                drive.followTrajectorySequence(pushAndReset1);

                Trajectory pushSample2 = drive.trajectoryBuilder(moveTowardsSamples.end())
                        .lineToLinearHeading(new Pose2d(moveTowardsSamples.end().getX() + 12, moveTowardsSamples.end().getY(), moveTowardsSamples.end().getHeading()))
                        .build();
                drive.followTrajectory(pushSample2);

                robot.moveIntake(-0.5);

                drive.followTrajectorySequence(pushAndReset1);

                robot.setIntakePosition(0.0);
                robot.setOutakePosition(0.0);

                Trajectory moveToSpecimens = drive.trajectoryBuilder(pushAndReset1.end())
                        .lineToLinearHeading(new Pose2d(33.5, 3, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(moveToSpecimens);

                robot.closeClaw();

                sleep(500);

                robot.setOutakePosition(5.0);

                Trajectory moveBack = drive.trajectoryBuilder(moveToSpecimens.end())
                        .lineToLinearHeading(new Pose2d(33.5, 12, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(moveBack);

                Trajectory moveToBar2 = drive.trajectoryBuilder(moveBack.end())
                        .lineToLinearHeading(new Pose2d(-3, 25, Math.toRadians(0)))
                        .build();
                drive.followTrajectory(moveToBar2);

                robot.encoderControl(robot.outakeMotor, DriveConstants.OUTAKE_MOTOR_STANDARD_SPEED, 19, DriveConstants.OUTAKE_COUNTS_PER_INCH);
                robot.encoderControl(robot.outakeMotor, DriveConstants.OUTAKE_MOTOR_STANDARD_SPEED, 19, DriveConstants.OUTAKE_COUNTS_PER_INCH);

                Trajectory moveIntoBar2 = drive.trajectoryBuilder(moveToBar2.end())
                        .lineToLinearHeading(new Pose2d(moveToBar2.end().getX(), moveToBar2.end().getY() + 3.5, 0))
                        .build();
                drive.followTrajectory(moveIntoBar2);

                robot.outakeMotor.setPower(-0.5);
                sleep(500);
                robot.outakeMotor.setPower(robot.outakeController.getF());
                robot.openClaw();

                break;
            case LEFT:

                break;
        }
    }
}
