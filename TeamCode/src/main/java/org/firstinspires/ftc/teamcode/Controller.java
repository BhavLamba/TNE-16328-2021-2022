package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
@Disabled
public class Controller extends LinearOpMode {
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;
//
//    private RobotHardware robot;
//    Toggle intake;
//    Orientation angle;
//
//
    @Override
    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetry.addData("Status", "Initializing...");
//        telemetry.update();
//
//        robot = new RobotHardware(hardwareMap, false);
//
//        telemetry.addData("Status", "Initialized");
//
//        telemetry.update();
//        waitForStart();
//
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//
//        intake = new Toggle(robot.motorIntake, 0.6);
//
//        while (opModeIsActive()) {
//            driveControl();
//            intakeControl();
//            servoControl();
//            dropControl();
////            timedServoControl();
//            spinnyControl();
//            telemetry.update();
//        }
    }
//
//    private void fieldCentricControl() {
//
//        angle = robot.imu.getAngularOrientation();
////        angle.firstAngle;
//        double radians = Math.toRadians(angle.firstAngle);
//
//
//        double scale = 0.5;
//        if (gamepad1.left_bumper) {
//            scale = 0.8;
//        } else if (gamepad1.left_trigger > 0.5) {
//            scale = 0.1;
//        }
//
//        double drive = gamepad1.left_stick_y;
//        double strafe = -gamepad1.left_stick_x;
//        double turn = -gamepad1.right_stick_x;
//
//        double temp = drive * Math.cos(radians) + strafe * Math.sin(radians);
//        strafe = -drive * Math.sin(radians) + strafe * Math.cos(radians);
//        drive = temp;
//
//        robot.driveTrain.startMove(drive, strafe, turn, scale);
//    }
//
//    private void driveControl() {
//        double scale = 0.5;
//        if (gamepad1.left_bumper) {
//            scale = 0.8;
//        } else if (gamepad1.left_trigger > 0.5) {
//            scale = 0.1;
//        }
//
//        double drive = gamepad1.left_stick_y;
//        double strafe = -gamepad1.left_stick_x;
//        double turn = -gamepad1.right_stick_x;
//        robot.driveTrain.startMove(drive, strafe, turn, scale);
//
//        telemetry.addData("BL pos", robot.driveTrain.motorBL.getCurrentPosition());
//        telemetry.addData("BR pos", robot.driveTrain.motorBR.getCurrentPosition());
//        telemetry.addData("FR pos", robot.driveTrain.motorFR.getCurrentPosition());
//        telemetry.addData("FL pos", robot.driveTrain.motorFL.getCurrentPosition());
//        telemetry.addData("BL pos", robot.driveTrain.motorBL.getCurrentPosition());

//    }

//    boolean isArmMovingUp = false;
//    boolean isArmMovingDown = false;
//    private void servoControl () {
//        if (gamepad1.b) {
//            isArmMovingUp = false;
//            isArmMovingDown = true;
//            robot.servoRArm.setPosition(robot.SERVO_LEFT_PICKUP_POS);
//            robot.servoLArm.setPosition(robot.SERVO_RIGHT_PICKUP_POS);
//            robot.servoRotate.setPosition(robot.SERVO_ROTATE_DODGE_MOTOR_POS);
//        } else if (gamepad1.y) {
//            isArmMovingUp = true;
//            isArmMovingDown = false;
//            robot.servoRArm.setPosition(robot.SERVO_LEFT_TOP_POS);
//            robot.servoLArm.setPosition(robot.SERVO_RIGHT_TOP_POS);
//        }
//        if (isArmMovingUp || isArmMovingDown) {
//            if (robot.servoLArm.getPosition() >= 0.15 && robot.servoRArm.getPosition() <= 0.7) {
//                robot.servoRotate.setPosition(robot.SERVO_ROTATE_DODGE_MOTOR_POS);
//            } else {
//                robot.servoRotate.setPosition(robot.SERVO_ROTATE_INIT_POS);
//            }
//            telemetry.addData("Left Arm Pos", robot.servoLArm.getPosition());
//            telemetry.addData("Rotate Pos", robot.servoRotate.getPosition());
//        }

//    }
//    private ElapsedTime armMoveTime = new ElapsedTime();
//    private double servoRPosition;
//    private double servoLPosition;
//    private double servoDelta = 0.5;
//    private double servoDelayMS = 5;
//    private boolean isArmMovingUp = false;
//    private boolean isArmMovingDown = false;
//    private void servoControl() {
//        if (gamepad1.y && armMoveTime.time() > servoDelayMS) {
//            isArmMovingUp = false;
//            isArmMovingDown = true;
//            servoRPosition = Range.clip(servoRPosition, robot.SERVO_RIGHT_PICKUP_POS,robot.SERVO_RIGHT_TOP_POS);
//            servoLPosition = Range.clip(servoLPosition,robot.SERVO_RIGHT_PICKUP_POS,robot.SERVO_RIGHT_TOP_POS);
//            servoLPosition += servoDelta;
//            servoRPosition -= servoDelta;
//            robot.servoRArm.setPosition(servoRPosition);
//            robot.servoLArm.setPosition(servoLPosition);
//            telemetry.addData("servoRPosition", servoRPosition);
//            telemetry.addData("servoLPosition", servoLPosition);
//            robot.servoRotate.setPosition(robot.SERVO_ROTATE_DODGE_MOTOR_POS);
//            armMoveTime.reset();
//        }
//        if (gamepad1.b) {
//            isArmMovingUp = true;
//            isArmMovingDown = false;
//            robot.servoRArm.setPosition(robot.SERVO_LEFT_TOP_POS);
//            robot.servoLArm.setPosition(robot.SERVO_RIGHT_TOP_POS);
//        }
//        if (isArmMovingUp || isArmMovingDown) {
//            if (robot.servoRArm.getPosition() >= 0.13 && robot.servoRArm.getPosition() <= 0.7) {
//                robot.servoRotate.setPosition(robot.SERVO_ROTATE_DODGE_MOTOR_POS);
//            } else {
//                robot.servoRotate.setPosition(robot.SERVO_ROTATE_INIT_POS);
//            }
//            telemetry.addData("Left Arm Pos", robot.servoLArm.getPosition());
//            telemetry.addData("Rotate Pos", robot.servoRotate.getPosition());
//        }
//    }
//
//
//    private boolean prevBumperPress = true;
//    private boolean isClawClose = true;
//    private void dropControl() {
//        if (gamepad1.right_bumper && !prevBumperPress) {
//            isClawClose = !isClawClose;
//            robot.setDrop(isClawClose);
//        }
//        prevBumperPress = gamepad1.right_bumper;
//    }
//
//    private void spinnyControl () {
//        if (gamepad1.dpad_up) {
//            robot.motorSpinny.setPower(0.5);
//        } else if (gamepad1.dpad_down) {
//            robot.motorSpinny.setPower(-0.5);
//        } else {
//            robot.motorSpinny.setPower(0);
//        }
//    }
//
//
//    private void intakeControl() {
//        intake.monitor(gamepad1.dpad_right);
//    }
//
//
//
//
//
//    private class Toggle {
//        boolean value = false;
//        boolean lastPress = false;
//        double power;
//        DcMotor motor;
//
//        private Toggle(DcMotor setMotor, double setPower) {
//            motor = setMotor;
//            power = setPower;
//        }
//
//        private void monitor(boolean input) {
//            if (input && !lastPress) {
//                value = !value;
//            }
//            lastPress = input;
//            if (value) {
//                motor.setPower(power);
//            } else {
//                motor.setPower(0);
//            }
//        }
//
//        private void swap() {
//            value = !value;
//        }
//    }
}

