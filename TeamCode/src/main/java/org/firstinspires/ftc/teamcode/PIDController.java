package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    ElapsedTime time;
    int error;
    int lastError;
    double derivative;
    double integralSum;

    int reference;

    

    public void run() {
        error =  reference - motor.currentPosition;
        derivative = (error - lastError) / timer.seconds();
        integralSum += (error * timer.seconds());

        motor.power = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;

        timer.reset();
    }
}
