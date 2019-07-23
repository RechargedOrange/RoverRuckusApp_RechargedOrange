package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Recharged Orange on 2/19/2019.
 */

@Autonomous (name = "PID_Depot")

public class PIDDepot extends superClass {



    public void runOpMode() {

        initialization(true);

        waitForStart(); //driver hits play

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        Deploy();

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        waitTimer.reset();

        while (waitTimer.seconds() < postDeployWait) {
            idle();
        }

        PID_driveForwardEncoders(600,.1,0);


        switch (goldPosition) {
            case LEFT:
                //drive to the left
                telemetry.addLine("Left");
                telemetry.update();
                driveleft(750, .1);
                PID_driveForwardEncoders(750,.1,0);
                pidTurn(0);
                PID_driveBackwardEncoders(750,.1,0);
                driveright(1830, .1);

                break;
            case RIGHT:

                telemetry.addLine("Right");
                telemetry.update();
                driveright(870, .1);
                PID_driveForwardEncoders(550,.1,0);
                PID_driveBackwardEncoders(430,.1,0);
                driveright(2650, .1);

                //drive to the right
                break;
            case CENTER:
                //drive straight

                telemetry.addLine("Forward, March");
                telemetry.update();
                PID_driveForwardEncoders(550,.1,0);
                PID_driveBackwardEncoders(560,.1,0);
                driveright(2200, .1);

                break;
            case UNKNOWN:

                telemetry.addLine("Unknown");
                telemetry.update();
                PID_driveForwardEncoders(550,.1,0);
                PID_driveBackwardEncoders(560,.1,0);
                driveright(2100, .1);

                //drive to the right
                break;

        }


        pidTurn(34);
        intakeLift.setPower(3);
        PID_driveForwardEncoders(2100,.2,34);
        sleep(750);
        TeamMarker.setPosition(TeamMarkerDOWN);
        sleep(300);
        intakeLift.setPower(0);
        PID_driveBackwardEncoders(2250,1,34);
        // driveBackwardEncoders(500,.1);
        sleep(30000);

    }

}
