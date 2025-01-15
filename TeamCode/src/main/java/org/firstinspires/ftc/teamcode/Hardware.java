package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
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
    public PIDFController outakeController = new PIDController(0.04, 0, 0.001);
    public double outakeTarget = 0;
    public double kF = 0.1;

    public boolean outakeIsBusy = false;

    public PIDController intakeController = new PIDController(0.01, 0, 0);
    public double intakeTarget = 0;

    public boolean intakeIsBusy = false;

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

    // rotates the intake servo by a specific amount using time
    public void rotateIntake(double speed, ElapsedTime deltaTime) {
        intakePitchVal += speed * deltaTime.time();

        if (intakePitchVal > 1) intakePitchVal = 1;
        if (intakePitchVal < 0) intakePitchVal = 0;

        intakePitch.setPosition(intakePitchVal);
    }

    public void rotateIntake(double degrees) {
        intakePitchVal += degrees / 360;

        setIntakePitch(intakePitchVal);
    }



    // sets the position of the intake (the whole thing connected to the viper slide, not the actual intake)
    public void setIntakePosition(double inches) {
        encoderControl(intakeMotor, INTAKE_MOTOR_STANDARD_SPEED, inches, INTAKE_COUNTS_PER_INCH);
    }
    // uses ticks instead of inches
    public void setIntakePosition(int ticks) {
        encoderControl(intakeMotor, INTAKE_MOTOR_STANDARD_SPEED, ticks);
    }

    // moves the entire intake x inches
    public void moveIntake(double inches) {
        int ticks = (int) (inches * INTAKE_COUNTS_PER_INCH);
        moveIntake(ticks);
    }
    // uses ticks instead of inches
    public void moveIntake(int ticks) {
        setIntakePosition(intakeMotor.getCurrentPosition() + ticks);
    }

    // sets the intakePitch servo position and changes the pitch value holder
    // use this function: DO NOT use intakePitch.setPosition (as it won't update intakePitchVal)
    public void setIntakePitch(double val) {
        intakePitch.setPosition(val);
        intakePitchVal = constrain(val, 0, 1);
    }

    // rotates the bucket by a specified amount
    // really only should be used when doing manual control of the bucket
    // speed input is in degrees/sec
    public void rotateBucket(double speed, ElapsedTime deltaTime) {
        bucketPitchVal += speed * deltaTime.time();

        setBucketPos(bucketPitchVal);
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

            encoderControl(motor, speed, newMotorTarget);
        }
    }
    // accepts integer values which signify the function is being given a position to go to in ticks and not inches
    public void encoderControl(DcMotor motor, double speed, int ticks) {
        if (op.opModeIsActive()) {
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(speed));

            while (op.opModeIsActive() && motor.isBusy()) {}

            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // sets the outake to a certain position in inches
    // assumes we are using encoders and not PID
    public void setOutakePosition(double inches) {
        encoderControl(outakeMotor, OUTAKE_MOTOR_STANDARD_SPEED, inches, OUTAKE_COUNTS_PER_INCH);
        outakeMotor.setPower(outakeController.getF());
    }

    public void setOutakePosition(double speed, double inches) {
        encoderControl(outakeMotor, speed, inches, OUTAKE_COUNTS_PER_INCH);
        outakeMotor.setPower(outakeController.getF());
    }

    // function only works if using PID for outake
    public void setOutakePosition(double inches, boolean usingPID) {
        if (usingPID) {
            int targetPos = (int) (inches * OUTAKE_COUNTS_PER_INCH);
            setOutakePosition(targetPos, true);
        }
        else {
            setOutakePosition(inches);
        }
    }
    // if target is an integer we assume it was given with units of encoder ticks (not inches)
    public void setOutakePosition(int ticks) {
        encoderControl(outakeMotor, OUTAKE_MOTOR_STANDARD_SPEED, ticks);
    }

    public void setOutakePosition(double speed, int ticks) {
        encoderControl(outakeMotor, speed, ticks);
    }
    // if target is an integer (given in encoder ticks) and we are using PID
    public void setOutakePosition(int ticks, boolean usingPID) {
        if (usingPID) {
            int outakePos = outakeMotor.getCurrentPosition();

            double power = outakeController.calculate(outakePos, ticks);

            outakeMotor.setPower(power);
        }
        else {
            setOutakePosition(ticks);
        }
    }

    // moves the outake a certain number of inches up/down
    public void moveOutake(double inches) {
        moveOutake((int) (inches * OUTAKE_COUNTS_PER_INCH));
    }

    public void moveOutake(double speed, double inches) {
        setOutakePosition(speed, (int) (inches * OUTAKE_COUNTS_PER_INCH));
    }
    // moves the outake a certain number of ticks up/down
    public void moveOutake(int ticks) {
        int newTarget = outakeMotor.getCurrentPosition() + ticks;
        setOutakePosition(newTarget);
    }

    public void openClaw() {
        claw.setPosition(ClawState.CLAW_OPEN.value);
    }
    public void closeClaw() {
        claw.setPosition(ClawState.CLAW_CLOSED.value);
    }

    public void updateOutake() {
        double pv = outakeMotor.getCurrentPosition() / OUTAKE_COUNTS_PER_INCH;
        double error = Math.abs(outakeTarget - pv);

        if (error <= 0.5) outakeIsBusy = false;
        else outakeIsBusy = true;

        double power = outakeController.calculate(outakeMotor.getCurrentPosition(), outakeTarget * OUTAKE_COUNTS_PER_INCH) + kF;

        outakeMotor.setPower(power);
    }

    public void updateIntake() {
        double power = intakeController.calculate(intakeMotor.getCurrentPosition() / INTAKE_COUNTS_PER_INCH, intakeTarget);

        intakeMotor.setPower(power);
    }
}