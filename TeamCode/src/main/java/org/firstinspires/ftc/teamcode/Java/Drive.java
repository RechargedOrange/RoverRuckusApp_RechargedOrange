package org.firstinspires.ftc.teamcode.Java;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.superClass;

/**
 * Created by me on 7/11/2019.
 */

@Config
public class Drive extends superClass {

    public static double kP = 0.007;
    public static double kI = 0.0;                              // these will be used in the PID methods
    public static double kD = 0.0;

    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightBack;

    double frontlefttarget = 0;
    double frontrighttarget = 0;
    double backrighttarget = 0;
    double backlefttarget = 0;// strings

    Orientation lastAngles = new Orientation();
    double globalAngle, turnPower = .30, correction;

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
