package org.firstinspires.ftc.teamcode;                                                  //This is to tell the phone where in Android Suudio the superClass is

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

/**
 * Created by Recharged Orange on 10/9/2018.
 */

@Config
public abstract class superClass extends linearOpmode {

    public static double kP = 0.007;
    public static double kI = 0.0;                              // these will be used in the PID methods
    public static double kD = 0.0;

    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightBack;                       // object statments
    public DcMotor lift;
    public DcMotor intake;
    public DcMotor intakeLift;

    public Servo Dumper;
    public Servo TeamMarker;

    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    public BNO055IMU imu;
    public RevTouchSensor liftTop;
    public RevTouchSensor liftBottom;               // more objects statments

    public float imuStartingPosition;
    double frontlefttarget = 0;
    double frontrighttarget = 0;
    double backrighttarget = 0;
    double backlefttarget = 0;// strings
    double TeamMarkerUP = 0;
    double TeamMarkerDOWN = .3;
    double DumperIn = .511;
    double DumperOut = .3156789;
    final String TELEOP_NAME = "MecanumDriveCode";
    boolean autonomous;
    double postDeployWait;
    boolean lastButtonState = false;
    public ElapsedTime waitTimer = new ElapsedTime();


    Orientation lastAngles = new Orientation();
    double globalAngle, turnPower = .30, correction;

    @Override
    // overriding the old waitForStart method
    public void waitForStart() {                                    // keeps it from uninting

        if (autonomous) {

            while (!isStarted() && !isStopRequested()) {
                telemetry.addData("We R Sampling", vision.getTfLite().getLastKnownSampleOrder());    // wait for the coach to hit play

                waitTime();
                telemetry.update();
            }
        } else
            while (!isStarted() && !isStopRequested()) {
                telemetry.addLine("Go Yeet It");    // wait for the coach to hit play
                telemetry.update();
            }

    }

    public void initialization(boolean autonomous) {

        this.autonomous = autonomous;

        telemetry.addLine("drive Init");
        telemetry.update();                                       //if there is a err we know what is an err
        initDrive();

        telemetry.addLine("lift init");
        telemetry.update();
        initlift();

        telemetry.addLine("init intake");
        telemetry.update();
        initintake();

        telemetry.addLine("init intakeLift");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("servo");
        telemetry.update();
        initServo();

        telemetry.addLine("REVTouch");
        telemetry.update();
        initREVTouch();

        if (autonomous) {
            telemetry.addLine("imu init");
            telemetry.update();
            initiate_imu();

            telemetry.addLine("initTFLite");
            telemetry.update();
            initTFLite();


            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        telemetry.addLine("Finished init - ready to go captain!");
        telemetry.update();
    }


    public void waitTime() {
        telemetry.addData("Wait time seconds", postDeployWait);
        telemetry.addLine("x to increment 1.0 seconds");
        telemetry.addLine("y to decrement 1.0 seconds");
        telemetry.addLine("lb to increment 0.1 seconds");
        telemetry.addLine("rb to decrement 0.1 seconds");

        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean lb = gamepad1.left_bumper;                                  // converts gamepad buttons into booleans
        boolean rb = gamepad1.right_bumper;
        boolean buttonState = x || y || lb || rb;
       //check button states
        if (buttonState && !lastButtonState) {
            if (x)
                postDeployWait += 1.0;
            if (y)
                postDeployWait -= 1.0;
            if (lb)
                postDeployWait += 0.1;
            if (rb)
                postDeployWait -= 0.1;
        }

        postDeployWait = Range.clip(postDeployWait, 0.0, 30.0);        //make sure that the wait numbers do not go below 0.0 and not above 30
                                                                                    // seconds

        lastButtonState = buttonState;                           // it resets the last button stated so it can be used next time

        telemetry.update();                                     // makes the drive coach able to see the numbers

    }

    public void initDrive() {

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");                      //runs all the motors when told
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // brakes it so it can stop quickly
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void Deploy() {
        while (!liftTop.isPressed() && opModeIsActive()) {                 // makes it posible to use in Autonomous
            lift.setPower(-1);                                            // these are called methods
        }

        lift.setPower(0);

        sleep(250);
        driveright(150, .1);
        PID_driveForwardEncoders(90,.1,0);
        driveleft(150, .1);
    }


    public void initlift() {
        lift = hardwareMap.dcMotor.get("lift");
    }

    public void initintake() {
        intake = hardwareMap.dcMotor.get("intake");
        intakeLift = hardwareMap.dcMotor.get("intakeLift");

    }

    public void initServo() {
        Dumper = hardwareMap.servo.get("Dumper");
        Dumper.setPosition(DumperOut);

        TeamMarker = hardwareMap.servo.get("TeamMarker");
        TeamMarker.setPosition(TeamMarkerUP);
    }

    public void initREVTouch() {
        liftTop = hardwareMap.get(RevTouchSensor.class, "liftTop");
        liftBottom = hardwareMap.get(RevTouchSensor.class, "liftBottom");
    }

    public void initTFLite() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AXi/CxP/////AAAAGV4xMjmD2EwntmuvBtxZnj8AOji5oAG2lxjzOJIGA9IASLd1EtX7KzZ6BpH6J0FWgEcjd8O/6mWD1rvLoAZ1R3KJcxH/xss+scSbd/U8d7/cZDupryfSH7lbRv94ZmPPwduAaQOkxyZfX0Gv+IsMUtIGqTZ5WIHYpqRSHIsGQQ6nlslCi5x/NRu0tnV1t6YgX6svoenYGXpbktnCYZB5BwO7OTfw7XrMMWtqSCJrd3PZha8rgiN1VvqvdEok//H0d9Vh5pnAMa8XwMEXx0N/0V1uEGUEcQvQA+fK7zghPqxjiXBQoZxcUUGkSbNGaIfTPBEoNoOi8QzHo4N6QN1TrgLnJW9J6tgbz9xzTpnRahqU";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_RIGHT);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time
    }

    public void initiate_imu() {

        telemetry.addLine("Initiating IMU");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (opModeIsActive()) {

            imuStartingPosition = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        }
    }

    public void driveForwardEncoders(double distance, double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);          //Turn on the motors to the specified power
        rightFront.setPower(power);

        backlefttarget = (leftBack.getCurrentPosition() + distance);
        backrighttarget = (rightBack.getCurrentPosition() + distance);
        frontlefttarget = (leftFront.getCurrentPosition() + distance);              //Create a number called motorNameTarget. This will be a bigger number
        frontrighttarget = (rightFront.getCurrentPosition() + distance);            //Than the current position of the motor, since the motor counts up as it goes forward

        while (leftBack.getCurrentPosition() < backlefttarget                   //while we have not reached the target we just set, keep running
                && leftFront.getCurrentPosition() < frontlefttarget
                && rightBack.getCurrentPosition() < backrighttarget
                && rightFront.getCurrentPosition() < frontrighttarget && opModeIsActive()) {

            telemetry.addData("left back", leftBack.getCurrentPosition());
            telemetry.addData("left front", leftFront.getCurrentPosition());
            telemetry.addData("right back", rightBack.getCurrentPosition());
            telemetry.addData("right front", rightFront.getCurrentPosition());

            telemetry.update();
        }
        driveOff();
    }

    public void pidTurn(double angle){
        pidTurn(angle, 2.0);
    }
    public void pidTurn(double angle, double threshold){
        IMUstraightDouble(angle);
        while(Math.abs(angle - getAngle()) > threshold && opModeIsActive()){
            double turn = IMUstraightDouble(angle);
            powerDriveTrain(turn, -turn);
        }
        DriveOff();
    }

    public void PID_driveForwardEncoders(double distance, double power, double angle) {

        backlefttarget = (leftBack.getCurrentPosition() + distance);
        backrighttarget = (rightBack.getCurrentPosition() + distance);
        frontlefttarget = (leftFront.getCurrentPosition() + distance);              //Create a number called motorNameTarget. This will be a bigger number
        frontrighttarget = (rightFront.getCurrentPosition() + distance);            //Than the current position of the motor, since the motor counts up as it goes forward


        IMUstraightDouble(angle);

        while (leftBack.getCurrentPosition() < backlefttarget                   //while we have not reached the target we just set, keep running
                && leftFront.getCurrentPosition() < frontlefttarget
                && rightBack.getCurrentPosition() < backrighttarget
                && rightFront.getCurrentPosition() < frontrighttarget && opModeIsActive()) {

            double turn = IMUstraightDouble(angle);
            powerDriveTrain((power + turn), (power - turn));
            telemetry.update();
        }


        DriveOff();
    }

    public void DriveOff(){
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);

    }

    PIDController pid = new PIDController(new PIDCoefficients(kP, kI, kD));

    public void powerDriveTrain(double leftPower, double rightPower) {

        double max = 1.0;
        max = Math.max(max, Math.abs(leftPower));
        max = Math.max(max, Math.abs(rightPower));
        leftPower /= max;
        rightPower /= max;

        leftBack.setPower(leftPower);
        leftFront.setPower(leftPower);
        rightBack.setPower(rightPower);
        rightFront.setPower(rightPower);

    }

    public double IMUstraightDouble(double targetAngle) {

        double currentAngle = getAngle();

        return pid.update(currentAngle - targetAngle);
    }

    public void driveBackwardEncoders(double distance, double power) {

        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);          //Turn on the motors to the specified power
        rightFront.setPower(-power);

        backlefttarget = (leftBack.getCurrentPosition() - distance);
        backrighttarget = (rightBack.getCurrentPosition() - distance);
        frontlefttarget = (leftFront.getCurrentPosition() - distance);              //Create a number called motorNameTarget. This will be a bigger number
        frontrighttarget = (rightFront.getCurrentPosition() - distance);

        while (leftBack.getCurrentPosition() > backlefttarget
                && leftFront.getCurrentPosition() > frontlefttarget
                && rightBack.getCurrentPosition() > backrighttarget
                && rightFront.getCurrentPosition() > frontrighttarget && opModeIsActive()) {

            telemetry.addData("left back", leftBack.getCurrentPosition());
            telemetry.addData("left front", leftFront.getCurrentPosition());
            telemetry.addData("right back", rightBack.getCurrentPosition());
            telemetry.addData("right front", rightFront.getCurrentPosition());

            telemetry.update();

        }
        driveOff();
    }

    public void PID_driveBackwardEncoders(double distance, double power, double angle) {
        backlefttarget = (leftBack.getCurrentPosition() - distance);
        backrighttarget = (rightBack.getCurrentPosition() - distance);
        frontlefttarget = (leftFront.getCurrentPosition() - distance);              //Create a number called motorNameTarget. This will be a bigger number
        frontrighttarget = (rightFront.getCurrentPosition() - distance);            //Than the current position of the motor, since the motor counts up as it goes forward


        IMUstraightDouble(angle);

        while (leftBack.getCurrentPosition() >  backlefttarget                   //while we have not reached the target we just set, keep running
                && leftFront.getCurrentPosition() > frontlefttarget
                && rightBack.getCurrentPosition() > backrighttarget
                && rightFront.getCurrentPosition() > frontrighttarget && opModeIsActive()) {

            double turn = IMUstraightDouble(angle);
            powerDriveTrain((-power + turn), (-power - turn));
            telemetry.update();
        }


        DriveOff();
    }

    public void driveleft(double distance, double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);

        backlefttarget = (leftBack.getCurrentPosition() + distance);
        backrighttarget = (rightBack.getCurrentPosition() + distance);
        frontlefttarget = (leftFront.getCurrentPosition() + distance);
        frontrighttarget = (rightFront.getCurrentPosition() - distance);


        while (backlefttarget >= leftBack.getCurrentPosition() && opModeIsActive()) {
            //Removed other encoder checks for now. ~Lane (Temp solution)

            telemetry.addData("left back", leftBack.getCurrentPosition());
            telemetry.addData("left front", leftFront.getCurrentPosition());
            telemetry.addData("right back", rightBack.getCurrentPosition());
            telemetry.addData("right front", rightFront.getCurrentPosition());

            telemetry.update();

        }
        DriveOff();
    }

    public void driveright(double distance, double power) {
        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(power);

        backlefttarget = (leftBack.getCurrentPosition() + distance);
        backrighttarget = (rightBack.getCurrentPosition() - distance);
        frontlefttarget = (leftFront.getCurrentPosition() - distance);
        frontrighttarget = (rightFront.getCurrentPosition() + distance);

        while (backrighttarget <= leftBack.getCurrentPosition() && opModeIsActive()) {
            telemetry.addData("left back", leftBack.getCurrentPosition());
            telemetry.addData("left front", leftFront.getCurrentPosition());
            telemetry.addData("right back", rightBack.getCurrentPosition());
            telemetry.addData("right front", rightFront.getCurrentPosition());

            telemetry.addData("Testing", frontlefttarget);

            telemetry.update();
        }

        DriveOff();
    }

    public void rotateLeft(double targetAngle, double power) {
        leftBack.setPower(-power);
        leftFront.setPower(-power);
        rightBack.setPower(power);
        rightFront.setPower(power);

        while (opModeIsActive() && getAngle() < targetAngle) {
            telemetry.addData("curentAngle", getAngle());
            telemetry.addData("targetAngle", targetAngle);
            telemetry.update();
        }
        DriveOff();
    }

    public void rotateRight(double targetAngle, double power) {
        leftBack.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(-power);
        rightFront.setPower(-power);

        while (opModeIsActive() && getAngle() > targetAngle) {
            telemetry.addData("curentAngle", getAngle());
            telemetry.addData("targetAngle", targetAngle);
            telemetry.update();
        }

       DriveOff();
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public void driveOff(){

    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
