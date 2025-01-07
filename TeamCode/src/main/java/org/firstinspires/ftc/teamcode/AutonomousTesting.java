package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name="Autonomous Testing", preselectTeleOp = "TeleOp")
public class AutonomousTesting extends LinearOpMode {
    Hardware robot = new Hardware();
    SampleMecanumDrive drive;

    public static double intakePos = 0;
    public static double outakePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.encoderControl(robot.intakeMotor, 0.6, intakePos, DriveConstants.INTAKE_COUNTS_PER_INCH);
            robot.encoderControl(robot.outakeMotor, 0.6, outakePos, DriveConstants.INTAKE_COUNTS_PER_INCH);
        }
    }
}
