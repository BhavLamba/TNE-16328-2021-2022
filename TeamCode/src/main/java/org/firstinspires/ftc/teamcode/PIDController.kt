package org.firstinspires.ftc.teamcode
//
//import com.qualcomm.robotcore.hardware.DcMotor
//import com.qualcomm.robotcore.util.ElapsedTime
//
//class PIDController(val kP: Double, val kI: Double, val kD: Double, val motor: DcMotor) {
//    private val timer = ElapsedTime();
//    private var error = 0
//    private var lastError = 0
//    private var derivative = 0.0
//    private var integralSum = 0.0
//    private var on = false
//    var reference = 0
//
//
//
//    fun run() {
//        error =  reference - motor.currentPosition
//        derivative = (error - lastError) / timer.seconds()
//        integralSum += (error * timer.seconds());
//
//        motor.power = (kP * error) + (kI * integralSum) + (kD * derivative)
//
//        lastError = error
//
//        timer.reset()
//    }
//
//}