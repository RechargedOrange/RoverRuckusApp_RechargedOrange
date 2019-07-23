package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp (name = "sample test")
@Disabled
public class VisionJavaExample extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AXi/CxP/////AAAAGV4xMjmD2EwntmuvBtxZnj8AOji5oAG2lxjzOJIGA9IASLd1EtX7KzZ6BpH6J0FWgEcjd8O/6mWD1rvLoAZ1R3KJcxH/xss+scSbd/U8d7/cZDupryfSH7lbRv94ZmPPwduAaQOkxyZfX0Gv+IsMUtIGqTZ5WIHYpqRSHIsGQQ6nlslCi5x/NRu0tnV1t6YgX6svoenYGXpbktnCYZB5BwO7OTfw7XrMMWtqSCJrd3PZha8rgiN1VvqvdEok//H0d9Vh5pnAMa8XwMEXx0N/0V1uEGUEcQvQA+fK7zghPqxjiXBQoZxcUUGkSbNGaIfTPBEoNoOi8QzHo4N6QN1TrgLnJW9J6tgbz9xzTpnRahqU";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_RIGHT);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        while(opModeIsActive()){
            goldPosition = vision.getTfLite().getLastKnownSampleOrder();
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }

            telemetry.update();
        }

        vision.shutdown();
    }
}
