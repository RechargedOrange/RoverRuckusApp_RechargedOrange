package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Recharged Orange on 1/29/2019.
 */

@Autonomous (name = "DepotSideOtherCrater")

public class DepotSideOtherCrater extends superClass {

    public void runOpMode() {

        initialization(true);

        waitForStart(); //driver hits play

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        Deploy();

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        waitTimer.reset();

        while (waitTimer.seconds() < postDeployWait) {idle();}

        driveForwardEncoders(600, .1);


        switch (goldPosition) {
            case LEFT:
                //drive to the left
                telemetry.addLine("Left");
                telemetry.update();
                driveleft(750, .1);
                driveForwardEncoders(1550, .1);
                rotateRight(-54,.05);
                driveForwardEncoders(500,.3);
                intakeLift.setPower(3);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                intakeLift.setPower(0);
                sleep(300);
                driveBackwardEncoders(2250, 1);
                sleep(30000);


                break;
            case RIGHT:

                telemetry.addLine("Right");
                telemetry.update();

                driveright(720, .1);
                driveForwardEncoders(1550, .1);
                rotateLeft(34,.05);
                driveForwardEncoders(1100,.2);
                intakeLift.setPower(3);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                intakeLift.setPower(0);
                rotateRight(-50,.05);
                sleep(300);
                driveBackwardEncoders(2250, 1);
                sleep(30000);


                //drive to the right
                break;
            case CENTER:
                //drive straight

                telemetry.addLine("Forward, March");
                telemetry.update();
                driveForwardEncoders(2050, .1);intakeLift.setPower(3);
                rotateRight(-48,.05);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                intakeLift.setPower(0);
                sleep(300);
                driveBackwardEncoders(2250, 1);
                sleep(30000);


                break;
            case UNKNOWN:

                telemetry.addLine("Unknown");
                telemetry.update();
                driveForwardEncoders(2050, .1);intakeLift.setPower(3);
                rotateRight(34,.05);
                sleep(750);
                TeamMarker.setPosition(TeamMarkerDOWN);
                intakeLift.setPower(0);
                sleep(300);
                driveBackwardEncoders(2050, 1);
                sleep(30000);


        }


    }

}