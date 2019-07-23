package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KNO3.AutoTransitioner;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

/**
 * Created by Recharged Orange on 11/8/2018.
 */

@Autonomous(name = "CraterSide")

public class CraterTest extends superClass {


    @Override
    public void runOpMode() {

        initialization(true);

        waitForStart();

        AutoTransitioner.transitionOnStop(this, TELEOP_NAME);

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        Deploy();

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        waitTimer.reset();

        while (waitTimer.seconds() < postDeployWait){idle(); }{
        }

        driveForwardEncoders(580, .1);


        switch (goldPosition) {
            case LEFT:
                //drive to the left

                telemetry.addLine("Left");
                telemetry.update();
                driveleft(850, .1);
                driveForwardEncoders(400, .1);
                rotateRight(10, .05);
                rotateLeft(0, .05);
                driveBackwardEncoders(200, .1);
                driveleft(2000, .1);

                break;
            case RIGHT:

                telemetry.addLine("Right");
                telemetry.update();

                driveright(750, .1);
                driveForwardEncoders(550, .1);
                driveBackwardEncoders(450, .1);
                //rotateRight(-10,.1);
                //rotateRight(0, .1);
                driveleft(750, .1);
                rotateRight(-10,.1);
                driveleft(2300, .1);

                //drive to the right
                break;
            case CENTER:
                //drive straight
                telemetry.addLine("Forward, March");
                telemetry.update();
                driveForwardEncoders(550, .1);
                driveBackwardEncoders(350, .1);
                rotateRight(-5,.1);
                driveleft(2600, .1);
                break;
            case UNKNOWN:

                telemetry.addLine("Unknown");
                telemetry.update();
                driveleft(870, .1);
                driveForwardEncoders(600, .1);
                rotateRight(0, .1);
                rotateLeft(0, .1);
                driveBackwardEncoders(300, .1);
                driveleft(3000, .1);

                //drive to the right
                break;

        }

        rotateLeft(110, .055);
        intakeLift.setPower(3);
        driveForwardEncoders(1000, .2);
        sleep(750);
        TeamMarker.setPosition(TeamMarkerDOWN);
        intakeLift.setPower(0);
        sleep(700);
        driveBackwardEncoders(3000, .24);   //tuned in to distance
        sleep(30000);

    }
}