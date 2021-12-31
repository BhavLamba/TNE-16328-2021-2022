package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    DcMotor motor;
    double kP, kI, kD;

    ElapsedTime timer;
    int error;
    int lastError;
    double derivative;
    double integralSum;

    int reference;

    public PIDController(double kp, double ki, double kd, DcMotor dcMotor) {
        kP = kp;
        kI = ki;
        kD = kd;
        motor = dcMotor;
        timer = new ElapsedTime();
    }

    public void run() {
        error =  reference - motor.getCurrentPosition();
        derivative = (error - lastError) / timer.seconds();
        integralSum += (error * timer.seconds());

        motor.setPower((kP * error) + (kI * integralSum) + (kD * derivative));

        lastError = error;

        timer.reset();
    }
}
