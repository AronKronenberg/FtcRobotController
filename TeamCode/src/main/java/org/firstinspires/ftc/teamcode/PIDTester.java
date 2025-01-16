package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class PIDTester extends LinearOpMode {
    Hardware robot = new Hardware();

    public static double kP = Hardware.outakeController.getP();
    public static double kI = Hardware.outakeController.getI();
    public static double kD = Hardware.outakeController.getD();
    public static double kF = Hardware.kF;

    public static double outakeTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            Hardware.outakeController.setPID(kP, kI, kD);
            Hardware.kF = kF;

            robot.setOutakeTarget(outakeTarget);

            double error = robot.updateOutake();

            telemetry.addData("target", outakeTarget * DriveConstants.OUTAKE_COUNTS_PER_INCH);
            telemetry.addData("pos", robot.outakeMotor.getCurrentPosition());
            telemetry.addData("error", error);

            telemetry.addLine();

            String isBusyState = robot.isOutakeBusy() ? "is busy" : "is not busy";
            telemetry.addData("Outake", isBusyState);

            telemetry.update();
        }
    }
}
