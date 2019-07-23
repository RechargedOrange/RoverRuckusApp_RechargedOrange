package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Recharged Orange on 9/27/2018.
 */
@TeleOp (name = "HappyForthOfJulyOnThe28th")

public class PresentationTele extends superClass {
    double drive;
    double strafe;
    double rotate;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    /**
     * if (going to crash)
     *  {
     *      don't
     *  };
     *   else
     *   {
     *       run amazing auto;
     *   }
     **/

    @Override
    public void runOpMode() {

        initialization(false);

        waitForStart();

        while (opModeIsActive()) {
            liftwithREVTouch();
            TeamMarker();
            intake();
            Dumper();
            intakeLift();
            MecanumDrive();
            telemetry.addLine("Running Program!");
            telemetry.update();
        }
    }


    public void MecanumDrive() {

        drive = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // You might have to play with the + or - depending on how your motors are installed
        frontLeftPower = drive + strafe + rotate;
        backLeftPower = drive - strafe + rotate;
        frontRightPower = drive - strafe - rotate;
        backRightPower = drive + strafe - rotate;

        leftBack.setPower(backLeftPower);
        leftFront.setPower(frontLeftPower);
        rightBack.setPower(backRightPower);
        rightFront.setPower(frontRightPower);                           //Adds the telemetry to the phone screen

    }


    public void Dumper(){

        if (gamepad2.right_bumper) {
            Dumper.setPosition(DumperIn);
        } else Dumper.setPosition(DumperOut);
    }

    public void intakeLift(){

        if (gamepad1.dpad_up){
            intakeLift.setPower(1);
        }else if (gamepad1.dpad_down) {
            intakeLift.setPower(-1);
        }
        else intakeLift.setPower(0);

    }


    public void liftwithREVTouch(){

        if (gamepad2.left_trigger > .5 && !liftBottom.isPressed()) {
            lift.setPower(1);
        }
        else if (gamepad2.right_trigger > .5 && !liftTop.isPressed()){
            lift.setPower(-1);
        }
        else {
            lift.setPower(0);
        }
    }

    public void intake() {
        if (gamepad1.right_trigger > 0.5) {
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.5)  {
            intake.setPower(-1);
        }
        else {intake.setPower(0);
        }
    }

    public void TeamMarker() {
        if (gamepad1.y){
            TeamMarker.setPosition(TeamMarkerDOWN);
        } else TeamMarker.setPosition(TeamMarkerUP);
    }

    }
