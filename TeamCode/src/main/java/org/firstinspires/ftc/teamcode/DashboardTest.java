package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Recharged Orange on 2/14/2019.
 */

@TeleOp
@Config
public class DashboardTest extends LinearOpMode{
    public static double val1 = 0.0;
    public static double val2 = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("val1", val1);
            telemetry.addData("val2", val2);
            telemetry.update();
        }
    }
}
