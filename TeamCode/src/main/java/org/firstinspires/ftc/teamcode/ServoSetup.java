package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class ServoSetup extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap, false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        Servo[] servos = new Servo[] {robot.servoRArm,     robot.servoLArm, robot.servoFlicker, robot.servoRotate};
        double[] pos = new double[] {robot.SERVO_RIGHT_INIT_POS, robot.SERVO_LEFT_INIT_POS, robot.SERVO_FLICKER_INIT_POS,   robot.SERVO_ROTATE_INIT_POS};
        String[] names = new String[] {"Right Arm",                 "Left Arm",             "Flicker",                       "Rotate"};

        int index = 0;
        boolean prevBumperPress = true;
        boolean prevButtonPress = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_bumper && !prevBumperPress) {
                index++;
                if (index >= servos.length) {
                    index = 0;
                }
            } else if (gamepad1.left_bumper && !prevBumperPress) {
                index--;
                if (index < 0) {
                    index = servos.length - 1;
                }
            }

            if (!prevButtonPress) {
                if (gamepad1.a) {
                    pos[index] = Range.clip(pos[index] - 0.05, -2, 2);
                    servos[index].setPosition(pos[index]);
                } else if (gamepad1.b) {
                    pos[index] = Range.clip(pos[index] + 0.05, -2, 2);
                    servos[index].setPosition(pos[index]);
                } else if (gamepad1.x) {
                    pos[index] = Range.clip(pos[index] - 0.01, -2, 2);
                    servos[index].setPosition(pos[index]);
                } else if (gamepad1.y) {
                    pos[index] = Range.clip(pos[index] + 0.01, -2, 2);
                    servos[index].setPosition(pos[index]);
                }
            }

            prevBumperPress = gamepad1.left_bumper || gamepad1.right_bumper;
            prevButtonPress = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;

            telemetry.addData("Servo", names[index]);
            telemetry.addData("Pos", pos[index]);
            telemetry.update();
        }
    }

}
