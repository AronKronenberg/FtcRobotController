package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DriveConstants {
    // *********** DRIVE *************
    public static final double FAST_SPEED = 1;
    public static final double NORMAL_SPEED = 0.75;
    public static final double SLOW_SPEED = 0.375;

    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.UP;




    // *********** MISC *************
    private static final double MM_TO_INCH = 1 / 25.4;



    // *********** INTAKE *************
    public static final double INTAKE_INIT_PITCH = 0.35;
    public static final double INTAKE_PITCH_SPEED = 1; // intake rotates this amount per second
    public static final double INTAKE_WHEEL_SPEED = 1;
    public static final double INTAKE_MOTOR_MAXIMUM_SPEED = 0.75;
    public static final double INTAKE_MOTOR_STANDARD_SPEED = 0.5625;

    // encoder values for intake
    private static final double INTAKE_DISTANCE_PER_ROTATION = 120 * MM_TO_INCH; // in / r
    private static final double INTAKE_MOTOR_COUNTS_PER_ROTATION = 751.8; // edit
    public static final double INTAKE_COUNTS_PER_INCH = 80.7432; //INTAKE_MOTOR_COUNTS_PER_ROTATION / (4.72496 * 2); //INTAKE_MOTOR_COUNTS_PER_ROTATION / INTAKE_DISTANCE_PER_ROTATION;
    public static final double MAXIMUM_INTAKE_COUNT = 15 * INTAKE_COUNTS_PER_INCH; // counts



    // *********** OUTAKE *************
    public static final double OUTAKE_MOTOR_MAXIMUM_SPEED = 0.75; //4.25; // units: inches / sec TODO: set this correctly
    public static final double OUTAKE_MOTOR_STANDARD_SPEED = 0.5625;

    // encoder values for outake
    public static final double MAXIMUM_OUTAKE_LENGTH = 976 * MM_TO_INCH; // in
    private static final double OUTAKE_DISTANCE_PER_ROTATION =  120 * MM_TO_INCH; // in / r
    private static final double OUTAKE_MOTOR_COUNTS_PER_ROTATION = 751.8; // edit
    public static final double OUTAKE_COUNTS_PER_INCH = 80.7432; //OUTAKE_MOTOR_COUNTS_PER_ROTATION / (4.72496 * 2); //OUTAKE_MOTOR_COUNTS_PER_ROTATION / OUTAKE_DISTANCE_PER_ROTATION;



    // *********** BUCKET *************
    // Positions of the bucket servo when intaking and outaking samples
    public static final double BUCKET_IN = 0.7;
    public static final double BUCKET_OUT = 0;

    public static final double BUCKET_PITCH_SPEED = 1;



    // *********** CLAW *************
    // Positions of the claw servo to pick up and drop specimens
    public static final double CLAW_OPEN = 0.8;
    public static final double CLAW_CLOSED = 0.35;
}