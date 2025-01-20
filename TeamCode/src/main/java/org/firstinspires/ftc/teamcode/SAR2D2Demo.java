package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.INTAKE_PITCH_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Demo")
public class SAR2D2Demo extends LinearOpMode {
    Hardware robot = new Hardware();
    ElapsedTime time;
    ElapsedTime deltaTime;

    boolean isTransferring = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        time = new ElapsedTime();
        deltaTime = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);

            if (!currentGamepad1.touchpad && previousGamepad1.touchpad) {
                isTransferring = true;
                time.reset();
            }
            if (isTransferring) {
                updateTransfer();
            }

            if (gamepad1.y && !isTransferring) { // move intake up
                robot.rotateIntake(-INTAKE_PITCH_SPEED, deltaTime);
            }
            else if (gamepad1.b && !isTransferring) { // move intake down
                robot.rotateIntake(INTAKE_PITCH_SPEED, deltaTime);
            }

            previousGamepad1.copy(gamepad1);

            deltaTime.reset();
        }
    }

    // automatically moves the intake towards the bucket and spits out a block
    public void updateTransfer() {
        telemetry.addData("Status", "Transferring...");
        telemetry.update();

        if (time.milliseconds() <= 750) {
            robot.setIntakePitch(0.3);
        }
        else if (time.milliseconds() <= 1250) {
            robot.intakeWheel.setPower(-1);
        }
        else if (time.milliseconds() <= 1500) {
            robot.intakeWheel.setPower(0);
            robot.setIntakePitch(0.55);
        }
        else if (time.milliseconds() <= 1750) {
            robot.setOutakeTarget(5.0);
            robot.updateOutake();
            robot.setBucketPos(0.4);
        }
        else {
            isTransferring = false;
        }
    }
}
