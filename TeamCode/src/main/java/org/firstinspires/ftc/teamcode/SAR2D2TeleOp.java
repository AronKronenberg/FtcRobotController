package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="TeleOp")
public class SAR2D2TeleOp extends LinearOpMode {
    Hardware robot = new Hardware();

    SampleMecanumDrive drive;

    YawPitchRollAngles angles;

    public static double driveSpeed = DriveConstants.NORMAL_SPEED;
    public static boolean fieldCentric = false;

    ElapsedTime deltaTime;
    ElapsedTime runtime;

    // gamepads
    Gamepad previousMain = new Gamepad();
    Gamepad previousSecondary = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        drive = new SampleMecanumDrive(hardwareMap);

        deltaTime = new ElapsedTime();
        runtime = new ElapsedTime();

        robot.intakePitch.setPosition(robot.intakePitchVal);
        robot.bucket.setPosition(robot.bucketPitchVal);

        setLEDColor(fieldCentric);

        // Use these when doing toggle input (idk why but they work better for that)
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // so telemetry is seen on ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized"); // tell the driver that initialization is complete

        waitForStart();

        if (isStopRequested()) return;

        runtime.reset();
        // runs every frame robot is active
        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // *********** DRIVING STUFF *************
            // for switching between field centric and robot centric drive
            // uses rising edge detection (only changes right when the button is clicked, not held)
            if (currentGamepad1.left_stick_button && !previousMain.left_stick_button) {
                fieldCentric = !fieldCentric;

                gamepad1.rumble(250);

                setLEDColor(fieldCentric);
            }

            // get the robot angles (yaw, pitch, roll)
            angles = robot.imu.getRobotYawPitchRollAngles();

            // controller input for driving
            double y = -gamepad1.left_stick_y; // positive --> forward, negative --> backwards
            double x = gamepad1.left_stick_x * 1.1; // positive --> right, negative --> left
            // (multiply by a little to account for strafe being slower)
            double rx = gamepad1.right_stick_x; // positive --> counterclockwise, negative --> clockwise

            drive(x, y, rx, fieldCentric); // does all the math and tells the motors to drive



            // *********** INTAKE STUFF *************
            // intake pitch control
            if (gamepad1.y) { // move intake up
                robot.rotateIntake(-DriveConstants.INTAKE_PITCH_SPEED, deltaTime);
            }
            else if (gamepad1.b) { // move intake down
                robot.rotateIntake(DriveConstants.INTAKE_PITCH_SPEED, deltaTime);
            }

            // automatic transfer from intake to outake
            // uses falling edge detection
            if (!currentGamepad1.touchpad && previousMain.touchpad) {
                transferSample();
            }

            // intake wheel control
            if (gamepad1.x) { // intake sample
                robot.intakeWheel.setPower(DriveConstants.INTAKE_WHEEL_SPEED);
            }
            else if (gamepad1.a) { // outake sample
                robot.intakeWheel.setPower(-DriveConstants.INTAKE_WHEEL_SPEED);
            }
            else {
                robot.intakeWheel.setPower(0);
            }

            // intake motor control (moves the linear slides forward/back and up/down)
            // moves intake out/in
            if (gamepad1.right_trigger > 0 && robot.intakeMotor.getCurrentPosition() < DriveConstants.MAXIMUM_INTAKE_COUNT) {
                // if statement has the last term to constrict the intake to stay within the limits (20x42 rectangle rule)
                robot.intakeMotor.setPower(DriveConstants.OUTAKE_MOTOR_MAXIMUM_SPEED * gamepad1.right_trigger); // pressure sensitive
            }
            else if (gamepad1.right_bumper) { // moves intake in (not pressure sensitive)
                robot.intakeMotor.setPower(-DriveConstants.INTAKE_MOTOR_STANDARD_SPEED);
            }
            else {
                robot.intakeMotor.setPower(0);
            }



            // *********** OUTAKE STUFF *************
            double outakeInput;
            if (gamepad1.left_trigger > 0) { // moves outake up (pressure sensitive)
                outakeInput = DriveConstants.OUTAKE_MOTOR_MAXIMUM_SPEED * gamepad1.left_trigger + robot.outakeController.getF();
                robot.outakeMotor.setPower(outakeInput);
            }
            else if (gamepad1.left_bumper) { // moves outake down (not pressure sensitive)
                outakeInput = -DriveConstants.OUTAKE_MOTOR_STANDARD_SPEED + robot.outakeController.getF();
                robot.outakeMotor.setPower(outakeInput);
            }
            else {
                outakeInput = robot.outakeController.getF();
                robot.outakeMotor.setPower(outakeInput);
            }

            // ************* BUCKET STUFF *************
            // There are two modes for the bucket: automatic and manual. This is code for automatic.
            if (currentGamepad1.dpad_up && !previousMain.dpad_up) {
                Hardware.BucketState.toggleState();
                robot.setBucketPos(Hardware.BucketState.getActiveState().getValue());
            }
            // manual
//            if (gamepad1.dpad_up) {
//                robot.rotateBucket(DriveConstants.BUCKET_PITCH_SPEED, deltaTime);
//            }
//            else if (gamepad1.dpad_down) {
//                robot.rotateBucket(-DriveConstants.BUCKET_PITCH_SPEED, deltaTime);
//            }

            // ************* CLAW STUFF *************
            if (currentGamepad1.dpad_right && !previousMain.dpad_right) {
                Hardware.ClawState.toggleState();
                robot.claw.setPosition(Hardware.ClawState.getActiveState().getValue());
            }

            // save gampepads
            previousMain.copy(currentGamepad1);
            previousSecondary.copy(currentGamepad2);

            // *********** TELEMETRY *************
            // send back data on current pos of outake and the target pos we want it to be at (both in inches)
            double outakePos = robot.outakeMotor.getCurrentPosition() / DriveConstants.OUTAKE_COUNTS_PER_INCH;

            telemetry.addData("Outake Input", outakeInput);
            telemetry.addData("\tPosition", outakePos + "\"");
            telemetry.addData("\tClaw State", Hardware.ClawState.getActiveState());

            telemetry.addLine();

            telemetry.addData("Bucket", "");
            //telemetry.addData("  Pitch Speed", DriveConstants.BUCKET_PITCH_SPEED + "d/s");
            telemetry.addData("\tState", Hardware.BucketState.getActiveState());


            telemetry.addLine();

            telemetry.addData("Intake", "");
            telemetry.addData("\tPosition", (robot.intakeMotor.getCurrentPosition() / DriveConstants.INTAKE_COUNTS_PER_INCH) + "\"");
            telemetry.addData("\tTarget Pitch", robot.intakePitchVal);

            telemetry.addLine();

            String driveMode = fieldCentric ? "Field Centric" : "Robot Centric";
            telemetry.addData("Drive Mode", driveMode);
            telemetry.addData("\tDrive input", y);
            telemetry.addData("\tStrafe input", x);
            telemetry.addData("\tTwist input", rx);

            // updates
            telemetry.update();
            deltaTime.reset();
        }
    }

    // default drive is robot centric if no boolean is included
    public void drive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.frontLeft.setPower(frontLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backLeft.setPower(backLeftPower);
        robot.backRight.setPower(backRightPower);
    }

    // if fieldCentric is true, will run field centric drive, otherwise it will run robot centric
    public void drive(double x, double y, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            double robotHeading = angles.getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
            double rotY = x * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);

            drive(rotX, rotY, rx);
        }
        else {
            drive(x, y, rx);
        }
    }

    // sets the led color of controller based on whether we are on field centric drive or not
    public void setLEDColor(boolean fieldCentric) {
        if (fieldCentric) gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        else gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    // automatically moves the intake towards the bucket and spits out a block
    public void transferSample() {
        telemetry.addData("Status", "Transferring...");
        telemetry.update();

        robot.setIntakePitch(0.3);
        sleep(750);
        robot.intakeWheel.setPower(-DriveConstants.INTAKE_WHEEL_SPEED);
        sleep(1000);
        robot.intakeWheel.setPower(0);
        robot.setIntakePitch(0.55);
        sleep(750);
        robot.setOutakePosition(5.0);
        robot.setBucketPos(0.4);
    }
}