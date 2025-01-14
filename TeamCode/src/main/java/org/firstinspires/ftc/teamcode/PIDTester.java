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
    public static double kP = 0.04;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double kF = 0.1;

    public static double target = 0;
    PIDController outakeController = new PIDController(kP, kI, kD);

    public static double kP2 = 0.01;
    public static double kI2 = 0;
    public static double kD2 = 0;

    public static double target2 = 0;
    PIDController intakeController = new PIDController(kP2, kI2, kD2);

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            outakeController.setPID(kP, kI, kD);
            intakeController.setPID(kP2, kI2, kD2);

            updateOutake(target);
            updateIntake(target2);

            telemetry.addData("target outake", target);
            telemetry.addData("current pos outake", robot.outakeMotor.getCurrentPosition() / DriveConstants.OUTAKE_COUNTS_PER_INCH);

            telemetry.addData("target intake", target2);
            telemetry.addData("current pos intake", robot.intakeMotor.getCurrentPosition() / DriveConstants.INTAKE_COUNTS_PER_INCH);

            telemetry.update();
        }
    }

    public void updateOutake(double target) {
        double power = outakeController.calculate(robot.outakeMotor.getCurrentPosition(), target * DriveConstants.OUTAKE_COUNTS_PER_INCH) + kF;

        robot.outakeMotor.setPower(power);
    }

    public void updateIntake(double target) {
        double power = intakeController.calculate(robot.intakeMotor.getCurrentPosition(), target2 * DriveConstants.INTAKE_COUNTS_PER_INCH);

        robot.intakeMotor.setPower(power);
    }
}
