package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KNO3.AutoTransitioner;

/**
 * Created by Recharged Orange on 2/18/2019.
 */

@Autonomous (name = "PID_Crater")

public class PIDCrater extends superClass {



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

        PID_driveForwardEncoders(580,.1,0);


        switch (goldPosition) {
            case LEFT:
                //drive to the left

                telemetry.addLine("Left");
                telemetry.update();
                driveleft(850, .1);
                PID_driveForwardEncoders(400,.1,0);
                pidTurn(0);
                PID_driveBackwardEncoders(200,.1,0);
                driveleft(2000, .1);

                break;
            case RIGHT:

                telemetry.addLine("Right");
                telemetry.update();

                driveright(750, .1);
                PID_driveForwardEncoders(550,.1,0);
                PID_driveBackwardEncoders(450,.1,0);
                driveleft(750, .1);
                pidTurn(-10);                                                       //Rotate so we don't hit the lander leg
                driveleft(2300, .1);

                //drive to the right
                break;
            case CENTER:
                //drive straight
                telemetry.addLine("Forward, March");
                telemetry.update();
                PID_driveForwardEncoders(550,.1,0);
                PID_driveBackwardEncoders(350,.1,0);
                pidTurn(-5);
                driveleft(2600, .1);
                break;
            case UNKNOWN:

                telemetry.addLine("Unknown");
                telemetry.update();
                PID_driveForwardEncoders(550,.1,0);
                PID_driveBackwardEncoders(350,.1,0);
                pidTurn(-5);
                driveleft(2600, .1);

                //drive to the right
                break;

        }

        pidTurn(135);
        intakeLift.setPower(3);
        PID_driveForwardEncoders(1000,.2,135);
        sleep(750);
        TeamMarker.setPosition(TeamMarkerDOWN);
        intakeLift.setPower(0);
        sleep(700);
        PID_driveBackwardEncoders(3000,.24,135);                      //tuned in to distance
        sleep(30000);

    }
}

