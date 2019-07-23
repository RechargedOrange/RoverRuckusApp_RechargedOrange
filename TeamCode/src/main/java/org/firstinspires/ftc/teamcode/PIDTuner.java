package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Recharged Orange on 2/14/2019.
 */

@TeleOp
public class PIDTuner extends superClass{
    public void runOpMode(){
        initialization(true);
        waitForStart();
        pidTurn(90.0);
        sleep(3000);
        pidTurn(-90.0);
        sleep(3000);
        pidTurn(0.0);
    }
}
