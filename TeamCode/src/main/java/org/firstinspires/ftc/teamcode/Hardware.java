package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware extends DriveConstants {
    HardwareMap hwMap;

    LinearOpMode op;

    public IMU imu;


    // *********** MOTORS *************
    // drive motors
    public DcMotor frontLeft; //3
    public DcMotor backLeft; //2
    public DcMotor frontRight; //1
    public DcMotor backRight; //0

    public DcMotor intakeMotor;
    public DcMotor outakeMotor;



    // *********** SERVOS *************
    public Servo intakePitch; // moves intake up and down

    public CRServo intakeWheel; // rotates the wheel to intake samples

    public Servo bucket;
    public enum BucketState {
        BUCKET_IN(DriveConstants.BUCKET_IN),
        BUCKET_OUT(DriveConstants.BUCKET_OUT);

        private double value;
        private static BucketState activeState = BUCKET_IN;

        BucketState(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public void setValue(double value) {
            this.value = value;
        }

        public static BucketState getActiveState() {
            return activeState;
        }

        public static void toggleState() {
            activeState = (activeState == BUCKET_IN) ? BUCKET_OUT : BUCKET_IN;
        }
    }

    public Servo claw;
    public enum ClawState {
        CLAW_OPEN(DriveConstants.CLAW_OPEN),
        CLAW_CLOSED(DriveConstants.CLAW_CLOSED);

        private double value;
        private static ClawState activeState = CLAW_OPEN;

        ClawState(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public void setValue(double value) {
            this.value = value;
        }

        public static ClawState getActiveState() {
            return activeState;
        }

        public static void toggleState() {
            activeState = (activeState == CLAW_OPEN) ? CLAW_CLOSED : CLAW_OPEN;
        }
    }



    // *********** USEFUL VARIABLES *************
    public PIDFController outakeController = new PIDFController(0.01, 0, 0, 0.1);

    // Non-hardware objects
    double intakePitchVal = INTAKE_INIT_PITCH;

    double bucketPitchVal = BUCKET_IN;



    // initialization method (could just be a constructor)
    public void init(LinearOpMode opMode) {
        // save reference to Hardware map
        op = opMode;
        hwMap = op.hardwareMap;

        // initialize the drivetrain
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");

        // initialize the intake servos
        intakePitch = hwMap.get(Servo.class, "intakePitch");
        intakeWheel = hwMap.get(CRServo.class, "intakeWheel");

        // initailize intake/outake motors (controls the linear slides)
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        outakeMotor = hwMap.get(DcMotor.class, "outakeMotor");

        bucket = hwMap.get(Servo.class, "bucket");
        claw = hwMap.get(Servo.class, "claw");

        // reverse the left-side drive motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // intakeMotor.setDirection(DcMotor.Direction.REVERSE); // maybe?
        outakeMotor.setDirection(DcMotor.Direction.REVERSE); // maybe?
        outakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize te gyro
        imu = hwMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        LOGO_FACING_DIR,
                        USB_FACING_DIR
                )
        );
        // use these parameters for the gyro
        imu.initialize(parameters);
    }

    // sets all the drive motors to a specific power
    public void setMotorPowers(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    // sets each drive motor to a specific power
    public void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    // sets the mode of all drive motors
    public void setDriveTrainMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }

    // rotates the intake servo by a specific amount
    public void rotateIntake(double speed, ElapsedTime deltaTime) {
        intakePitchVal += speed * deltaTime.time();

        if (intakePitchVal > 1) intakePitchVal = 1;
        if (intakePitchVal < 0) intakePitchVal = 0;

        intakePitch.setPosition(intakePitchVal);
    }



    // moves the entire intake a certain number of inches
    public void setIntakePos(double speed, double inches) {
        encoderControl(this.intakeMotor, speed, inches, INTAKE_COUNTS_PER_INCH);
    }

    // sets the intakePitch servo position and changes the pitch value holder
    // use this function: DO NOT use intakePitch.setPosition (as it won't update intakePitchVal)
    public void setIntakePitch(double val) {
        intakePitch.setPosition(val);
        intakePitchVal = constrain(val, 0, 1);
    }

    // rotates the bucket by a specified amount
    public void rotateBucket(double speed, ElapsedTime deltaTime) {
        bucketPitchVal += speed * deltaTime.time();

        bucketPitchVal = constrain(bucketPitchVal, 0, 1);

        bucket.setPosition(bucketPitchVal);
    }

    // sets the bucket servo position and changes our pitch value holder
    // use this function whenever setting the bucket value. DO NOT use bucket.setPosition()
    public void setBucketPos(double pos) {
        bucket.setPosition(pos);
        bucketPitchVal = constrain(pos, 0, 1);
    }

    public double constrain(double n, double min, double max) {
        if (n > max) return max;
        if (n < min) return min;
        return n;
    }

    // general function to move a motor x inches using encoders
    public void encoderControl(DcMotor motor, double speed, double inches, double COUNTS_PER_INCH) {
        int newMotorTarget;

        if (op.opModeIsActive()) {
            newMotorTarget = (int) (inches * COUNTS_PER_INCH);

            motor.setTargetPosition(newMotorTarget);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(speed));

            while (op.opModeIsActive() && motor.isBusy()) {

            }
            motor.setPower(0);
            if (motor.equals(outakeMotor)) {
                motor.setPower(outakeController.getF());
            }
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // sets the outake to a certain position in inches
    public void setOutakePosition(double target) {
        int outakePos = outakeMotor.getCurrentPosition();
        int targetPos = (int) (target * OUTAKE_COUNTS_PER_INCH);

        double power = outakeController.calculate(outakePos, targetPos);

        outakeMotor.setPower(power);
    }
    // if target is an integer we assume it was given with units of encoder ticks (not inches)
    public void setOutakePosition(int target) {
        int outakePos = outakeMotor.getCurrentPosition();

        double power = outakeController.calculate(outakePos, target);

        outakeMotor.setPower(power);
    }

    // moves the outake a certain number of inches up/down
    public void moveOutake(double inches) {
        // convert current position to inches and add the # of inches we want to move the outake
        double target = outakeMotor.getCurrentPosition() * (1 / OUTAKE_COUNTS_PER_INCH) + inches;
        setOutakePosition(target);
    }
}