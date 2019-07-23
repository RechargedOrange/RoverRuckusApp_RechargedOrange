package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Recharged Orange on 2/19/2019.
 */

@Autonomous (name = "PID_DepotOtherCrater")

public class PID_DepotOtherCrater extends superClass {

    public void runOpMode() {

        initialization(true);

        waitForStart(); //driver hits play

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        Deploy();

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        waitTimer.reset();

        while (waitTimer.seconds() < postDeployWait) {idle();}

        PID_driveForwardEncoders(600,.1,0);


        switch (goldPosition) {
            case LEFT:
                //drive to the left
                telemetry.addLine("Left");
                telemetry.update();
                driveleft(750, .1);
                PID_driveForwardEncoders(1550,.1,0);
                pidTurn(-54);
                PID_driveForwardEncoders(500,.3,-54);
                intakeLift.setPower(3);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                intakeLift.setPower(0);
                sleep(300);
                PID_driveBackwardEncoders(2450,1,-54);
                sleep(30000);


                break;
            case RIGHT:

                telemetry.addLine("Right");
                telemetry.update();

                driveright(720, .1);
                PID_driveForwardEncoders(1550,.1,0);
                pidTurn(34);
                PID_driveForwardEncoders(1150,.2,34);
                intakeLift.setPower(3);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                pidTurn(-53);
                intakeLift.setPower(0);
                sleep(300);
                PID_driveBackwardEncoders(2480,1,-50);
                sleep(30000);


                //drive to the right
                break;
            case CENTER:
                //drive straight

                telemetry.addLine("Forward, March");
                telemetry.update();
                PID_driveForwardEncoders(2150,.1,0);
                intakeLift.setPower(3);
                pidTurn(-49);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                intakeLift.setPower(0);
                sleep(300);
                PID_driveBackwardEncoders(2950,.1,-48);
                sleep(30000);


                break;
            case UNKNOWN:

                telemetry.addLine("Unknown");
                telemetry.update();
                driveright(720, .1);
                PID_driveForwardEncoders(1550,.1,0);
                pidTurn(34);
                PID_driveForwardEncoders(1100,.2,34);
                intakeLift.setPower(3);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                intakeLift.setPower(0);
                pidTurn(-50);
                sleep(300);
                PID_driveBackwardEncoders(2250,1,-50);
                sleep(30000);



        }


    }

}


