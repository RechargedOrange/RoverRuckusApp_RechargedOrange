package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer

/**
 * Created by Recharged Orange on 12/3/2018.
 */
@TeleOp
@Disabled
class DavidVisionTest : LinearOpMode(){
    override fun runOpMode() {
        val params = VuforiaLocalizer.Parameters()
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
        params.vuforiaLicenseKey = "AXi/CxP/////AAAAGV4xMjmD2EwntmuvBtxZnj8AOji5oAG2lxjzOJIGA9IASLd1EtX7KzZ6BpH6J0FWgEcjd8O/6mWD1rvLoAZ1R3KJcxH/xss+scSbd/U8d7/cZDupryfSH7lbRv94ZmPPwduAaQOkxyZfX0Gv+IsMUtIGqTZ5WIHYpqRSHIsGQQ6nlslCi5x/NRu0tnV1t6YgX6svoenYGXpbktnCYZB5BwO7OTfw7XrMMWtqSCJrd3PZha8rgiN1VvqvdEok//H0d9Vh5pnAMa8XwMEXx0N/0V1uEGUEcQvQA+fK7zghPqxjiXBQoZxcUUGkSbNGaIfTPBEoNoOi8QzHo4N6QN1TrgLnJW9J6tgbz9xzTpnRahqU"
        val vision = MasterVision(params, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_RIGHT)
        vision.init()
        vision.enable()
        waitForStart()
        while(opModeIsActive()){
            telemetry.addData("order", vision.tfLite.lastKnownSampleOrder)
            telemetry.update()
        }
    }
}