package org.firstinspires.ftc.teamcode.Java;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.superClass;

/**
 * Created by Recharged Orange on 6/13/2019.
 */

@Autonomous(name = "The_Auto_That_Nate_Deleted")

public class The_Auto_that_Nate_Deleted extends superClass {

    @Override

    public void runOpMode(){

        initialization(true);

        waitForStart();

        driveForwardEncoders(750,.2);

        driveBackwardEncoders(750,.2);
    }

}
