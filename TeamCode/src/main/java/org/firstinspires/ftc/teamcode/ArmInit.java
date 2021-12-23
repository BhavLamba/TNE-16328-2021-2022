package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ArmInit extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.servoRotate.setPosition(robot.SERVO_ROTATE_DODGE_MOTOR_POS);
        sleep(1000);
        robot.servoLArm.setPosition(robot.SERVO_LEFT_PICKUP_POS);
        robot.servoRArm.setPosition(robot.SERVO_RIGHT_PICKUP_POS);
        sleep(1000);
        robot.servoRotate.setPosition(robot.SERVO_ROTATE_PICKUP_POS);
        sleep(1000);
        robot.servoFlicker.setPosition(robot.SERVO_FLICKER_CLOSE_POS);

    }
}