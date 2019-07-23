package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

/**
 * Created by Recharged Orange on 10/18/2018.
 */

@Autonomous (name = "DepotSide")

public class DepoTest extends superClass {

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

        driveForwardEncoders(600, .1);


        switch (goldPosition) {
            case LEFT:
                //drive to the left
                telemetry.addLine("Left");
                telemetry.update();
                driveleft(750, .1);
                driveForwardEncoders(750, .1);
                rotateLeft(0, .1);
                rotateRight(0, .1);
                driveBackwardEncoders(750, .1);
                driveright(1910, .1);


       /* driveleft(750,.1);
        driveForwardEncoders(550,.1);1650
        rotateRight(0,.1);
        rotateLeft(0,.1);
        driveBackwardEncoders(550,.1);
        driveright(750,.1);*/

                break;
            case RIGHT:

                telemetry.addLine("Right");
                telemetry.update();

                driveright(720, .1);
                driveForwardEncoders(550, .1);
                // rotateLeft(0,.1);
                // rotateRight(0,.1);
                driveBackwardEncoders(430, .1);
                driveright(2650, .1);

                //drive to the right
                break;
            case CENTER:
                //drive straight

                telemetry.addLine("Forward, March");
                telemetry.update();
                driveForwardEncoders(550, .1);
                // rotateLeft(0,.1);
                //rotateRight(0,.1);h
                driveBackwardEncoders(550, .1);
                driveright(2100, .1);

                break;
            case UNKNOWN:

                telemetry.addLine("Unknown");
                telemetry.update();
                driveForwardEncoders(550, .1);
                // rotateLeft(0,.1);
                //rotateRight(0,.1);
                driveBackwardEncoders(550, .1);
                driveright(2100, .1);

                //drive to the right
                break;

        }


        rotateLeft(34, .05);
        intakeLift.setPower(3);
        //sleep(300);
        driveForwardEncoders(2300, .2);
        sleep(750);
        TeamMarker.setPosition(TeamMarkerDOWN);
        sleep(300);
        intakeLift.setPower(0);
        driveBackwardEncoders(2950, 1);
        // driveBackwardEncoders(500,.1);
        sleep(30000);

    }

}
