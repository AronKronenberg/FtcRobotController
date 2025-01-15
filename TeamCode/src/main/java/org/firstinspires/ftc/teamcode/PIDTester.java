package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class PIDTester extends LinearOpMode {
    Hardware robot = new Hardware();

    public static double intakeTarget;
    public static double outakeTarget;

    boolean isBusyAtAll = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            robot.outakeTarget = outakeTarget;

            robot.updateOutake();
            // updateIntake(intakeTarget);

            telemetry.addData("target outake", outakeTarget);
            telemetry.addData("current pos outake", robot.outakeMotor.getCurrentPosition() / DriveConstants.OUTAKE_COUNTS_PER_INCH);

            telemetry.addData("target intake", intakeTarget);
            telemetry.addData("current pos intake", robot.intakeMotor.getCurrentPosition() / DriveConstants.INTAKE_COUNTS_PER_INCH);

            if (robot.outakeIsBusy) {
                isBusyAtAll = true;
            }

            telemetry.addData("Outake is busy", robot.outakeIsBusy);
            telemetry.addData("Is Busy", isBusyAtAll);

            telemetry.update();
        }
    }

    public void updateOutake(double target) {
        double power = robot.outakeController.calculate(robot.outakeMotor.getCurrentPosition(), target * DriveConstants.OUTAKE_COUNTS_PER_INCH) + robot.kF;

        robot.outakeMotor.setPower(power);
    }

    public void updateIntake(double target) {
        double power = robot.intakeController.calculate(robot.intakeMotor.getCurrentPosition(), target * DriveConstants.INTAKE_COUNTS_PER_INCH);

        robot.intakeMotor.setPower(power);
    }
}
