package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class Controller extends LinearOpMode {
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    private RobotHardware robot;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap, false);

        telemetry.addData("Status", "Initialized");

        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            driveControl();
            telemetry.update();
        }
    }


    private void driveControl() {
        double scale = 0.6;
        if (gamepad1.left_bumper) {
            scale = 1.0;
        } else if (gamepad1.left_trigger > 0.5) {
            scale = 0.3;
        }

        double drive = -gamepad1.left_stick_y; // ?? why is this getting negated
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        robot.startMove(drive, strafe, turn, scale);
    }
}

