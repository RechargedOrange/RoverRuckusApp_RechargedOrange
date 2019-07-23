package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by Recharged Orange on 2/4/2019.
 */

public class PIDController {
    private final PIDCoefficients pidCoefficients;

    private double integral = 0.0;
    private long lastTime = 0;
    private double lastError = 0.0;

    public double update(double error){
        long currTime = System.currentTimeMillis();
        double dt = (currTime - lastTime) / 1000.0;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastTime = currTime;
        lastError = error;
        return error * pidCoefficients.p + integral * pidCoefficients.i + derivative * pidCoefficients.d;
    }

    public PIDController(PIDCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
    }
}
