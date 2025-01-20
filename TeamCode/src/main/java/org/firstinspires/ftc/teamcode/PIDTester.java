package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.DriveConstants.*;

@Config
@TeleOp
public class PIDTester extends LinearOpMode {
    Hardware robot = new Hardware();

    public static double kP = Hardware.outakeController.getP();
    public static double kI = Hardware.outakeController.getI();
    public static double kD = Hardware.outakeController.getD();
    public static double kF = Hardware.kF;

    public static double outakeTarget;

    public static double targetIncrease = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            currentGamepad.copy(gamepad1);

            Hardware.outakeController.setPID(kP, kI, kD);
            Hardware.kF = kF;

            time.reset();
            while (gamepad1.dpad_up) {
                if (time.milliseconds() >= 500) {
                    outakeTarget += targetIncrease;
                    telemetry.addData("target", outakeTarget * OUTAKE_COUNTS_PER_INCH);
                    telemetry.update();
                    time.reset();
                }
            }
            while (gamepad1.dpad_down) {
                if (time.milliseconds() >= 500) {
                    outakeTarget -= targetIncrease;
                    telemetry.addData("target", outakeTarget * OUTAKE_COUNTS_PER_INCH);
                    telemetry.update();
                    time.reset();
                }
            }

            robot.setOutakeTarget(outakeTarget);

            double error = robot.updateOutake();

            telemetry.addData("target", outakeTarget);
            telemetry.addData("pos", robot.outakeMotor.getCurrentPosition() / OUTAKE_COUNTS_PER_INCH);
            telemetry.addData("error", error);

            telemetry.addLine();

            String isBusyState = robot.isOutakeBusy() ? "is busy" : "is not busy";
            telemetry.addData("Outake", isBusyState);

            telemetry.update();
        }
    }
}
