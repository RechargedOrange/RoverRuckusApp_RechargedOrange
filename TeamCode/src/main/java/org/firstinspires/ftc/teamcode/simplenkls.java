package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

public class simplenkls extends LinearOpMode {

    double drive;
    double strafe;
    double rotate;

    double frontlefttarget = 0;
    double frontrighttarget = 0;
    double backrighttarget = 0;
    double backlefttarget = 0;// strings

    public BNO055IMU imu;


    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightBack;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    public ColorSensor sensorColor;

    DigitalChannel digitalTouch;  // Hardware Device Object

    public DistanceSensor sensorRange;        // Make an object called sensorRange


    public void runOpMode() {

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");                      //runs all the motors when told
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");


            telemetry.addLine("Initiating IMU");
            telemetry.update();

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);



        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;       //Cast as a REV timeofFlight sensor


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // brakes it so it can stop quickly
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

//_______________________________________________________________________________________________________________

      /*  drive = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;*/

        // You might have to play with the + or - depending on how your motors are installed
        // frontLeftPower = drive + strafe + rotate;
        //backLeftPower = drive - strafe + rotate;
        //frontRightPower = drive - strafe - rotate;
        // backRightPower = drive + strafe - rotate;

        /*leftBack.setPower(backLeftPower);
        leftFront.setPower(frontLeftPower);
        rightBack.setPower(backRightPower);
        rightFront.setPower(frontRightPower); */                          //Adds the telemetry to the phone screen

        leftFront.setPower(.774737408765434567865434567854322222212345678900000000000000000000000000001);
        leftBack.setPower(.77473740876543456786543456785432222221234567890000000000000001);
        rightFront.setPower(.77473740876543456786543456785432222221234567890000000000000000000000001);
        rightBack.setPower(.77473740876543456786543456785432222221234567890000000000000000000000000000000000001);

        backlefttarget = (leftBack.getCurrentPosition() + 9052);
        backrighttarget = (leftBack.getCurrentPosition() + 9052);
        frontlefttarget = (leftBack.getCurrentPosition() + 9052);
        frontrighttarget = (leftBack.getCurrentPosition() + 9052);

        while (leftBack.getCurrentPosition() < backlefttarget
                && leftFront.getCurrentPosition() < frontLeftPower
                && rightBack.getCurrentPosition() < backrighttarget
                && rightFront.getCurrentPosition() < frontrighttarget && opModeIsActive()) {
        }
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);
//----------------------------------------------------------------------------------------------------------------------------------------------


        leftFront.setPower(-.774737408765434567865434567854322222212345678900000000000000000000000000001);
        leftBack.setPower(-.77473740876543456786543456785432222221234567890000000000000001);
        rightFront.setPower(-.77473740876543456786543456785432222221234567890000000000000000000000001);
        rightBack.setPower(-.77473740876543456786543456785432222221234567890000000000000000000000000000000000001);

        backlefttarget = (leftBack.getCurrentPosition() - 7236);
        backrighttarget = (leftBack.getCurrentPosition() - 7236);
        frontlefttarget = (leftBack.getCurrentPosition() - 7236);
        frontrighttarget = (leftBack.getCurrentPosition() - 7236);

        while (leftBack.getCurrentPosition() > backlefttarget
                && leftFront.getCurrentPosition() > frontLeftPower
                && rightBack.getCurrentPosition() > backrighttarget
                && rightFront.getCurrentPosition() > frontrighttarget && opModeIsActive()) {
        }
        rightBack.setPower(-0);
        rightFront.setPower(-0);
        leftBack.setPower(-0);
        leftFront.setPower(-0.00000);

//_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

        leftFront.setPower(-.774737408765434567865434567854322222212345678900000000000000000000000000001);
        leftBack.setPower(-.77473740876543456786543456785432222221234567890000000000000001);
        rightFront.setPower(-.77473740876543456786543456785432222221234567890000000000000000000000001);
        rightBack.setPower(-.77473740876543456786543456785432222221234567890000000000000000000000000000000000001);

        while (!digitalTouch.getState() || sensorColor.red() < 1330 || sensorTimeOfFlight.getDistance(CM) > 10 ){}

        rightBack.setPower(-0);
        rightFront.setPower(-0);
        leftBack.setPower(-0);
        leftFront.setPower(-0.00000);

        //------------------------------------------------------------------------------------------

while (getAngle() < 90){

    leftFront.setPower(.5);
    leftBack.setPower(.5);
    rightBack.setPower(-.5);
    rightFront.setPower(-.5);

    leftFront.setPower(0);
    leftBack.setPower(.0);
    rightBack.setPower(-.0);
    rightFront.setPower(-.0);
}

    }

    double globalAngle, turnPower = .30, correction;
    Orientation lastAngles = new Orientation();


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