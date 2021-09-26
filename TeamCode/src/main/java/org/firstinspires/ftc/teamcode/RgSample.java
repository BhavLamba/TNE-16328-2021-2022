package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp

public class RgSample extends LinearOpMode {
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
            shooterControl();
            intakeControl();
            angleControl();
            armControl();
            clawControl();
            telemetry.update();
        }
    }

    private boolean wasRightDpadPressed = true;
    private boolean isIntakeOn = false;

    private void intakeControl() {
        if (gamepad1.dpad_right && !wasRightDpadPressed) {
            SetIntake(!isIntakeOn);
        } else if (gamepad1.dpad_left) {
            robot.motorIntake.setPower(-1);
        }
        telemetry.addData("Intake Pos", robot.motorIntake.getCurrentPosition());
        wasRightDpadPressed = gamepad1.dpad_right;
    }

    private void SetIntake(boolean setIntakeOn) {
        isIntakeOn = setIntakeOn;
        if (isIntakeOn) {
            robot.motorIntake.setPower(0.8);
        } else {
            robot.motorIntake.setPower(0);
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

    private boolean prevXPress = true;
    private boolean isClawClose = true;

    private void clawControl() {
        if (gamepad1.x && !prevXPress) {
            isClawClose = !isClawClose;
            robot.setClaw(isClawClose);
        }
        prevXPress = gamepad1.x;
    }


    private boolean isShooterOn = false;
    private boolean wasPressingRightTrigger = true;

    private enum RingPushStates {IN, MOVING_OUT, MOVING_IN}

    private ElapsedTime ringPushTime = new ElapsedTime();
    private RingPushStates ringPushState = RingPushStates.IN;

    private void shooterControl() {
//        if (gamepad1.right_trigger > 0.5 && !wasPressingRightTrigger) {
//            if (isShooterOn && ringPushState == RingPushStates.IN) {
//                isShooterOn = false;
//                robot.motorShooter.setPower(0);
//            } else if (isHoldingShooterAngle) {
//                isShooterOn = true;
//                robot.motorShooter.setPower(-1);
//            }
//        }

        if (ringPushState == RingPushStates.MOVING_IN && ringPushTime.milliseconds() > robot.PUSH_MOVE_IN_TIME_MS) {
            ringPushState = RingPushStates.IN;
        }
        if (gamepad1.right_bumper && ringPushState == RingPushStates.IN) {
            robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
            ringPushState = RingPushStates.MOVING_OUT;
            ringPushTime.reset();
        } else if (ringPushState == RingPushStates.MOVING_OUT && ringPushTime.milliseconds() > robot.PUSH_MOVE_OUT_TIME_MS) {
            robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
            ringPushState = RingPushStates.MOVING_IN;
            ringPushTime.reset();
        }

//        if (gamepad2.right_trigger > 0.5) {
//            powerShotsSetup();
//        } else if (gamepad2.dpad_left) {
//            shootRings();
//        }

        telemetry.addData("Shooter Pos", robot.motorShooter.getCurrentPosition());
        telemetry.addData("Shooter Power", robot.motorShooter.getPower());
//        wasPressingRightTrigger = gamepad1.right_trigger > 0.5;

    }

    private void armControl() {
        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.motorArm.setPower(-0.7);
        } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.motorArm.setPower(0.9);
        } else {
            robot.motorArm.setPower(0);
        }

        telemetry.addData("Arm Pos", robot.motorArm.getCurrentPosition());

    }

    private double angleShootPos = robot.ANGLE_SHOOT_HIGH_POS;
    private double angleHoldPos = robot.HOLD_HIGH_GOAL_POS;
    private boolean isHoldingShooterAngle = false;
    private boolean isAngleDown = false;
//    private ElapsedTime angleUpTime = new ElapsedTime();
    private void angleControl () {
        if (gamepad1.y || gamepad2.y && !isShooterOn) {
            isHoldingShooterAngle = true;
            angleShootPos = robot.ANGLE_SHOOT_HIGH_POS;
            robot.motorAngle.setPower(-0.1);
            angleHoldPos = robot.HOLD_HIGH_GOAL_CLOSE_POS;
            robot.servoHolder.setPosition(angleHoldPos);
            robot.motorShooter.setPower(-1);
            SetIntake(false);
        } else if (gamepad1.b || gamepad2.b && ringPushState == RingPushStates.IN) {
            isHoldingShooterAngle = false;
            isShooterOn = false;
            isAngleDown = true;
            robot.servoHolder.setPosition(robot.HOLD_ERECT_POS);
            robot.motorShooter.setPower(0);
            SetIntake(true);
            moveAngleUp(50);
        } else if (gamepad1.right_trigger > 0.5 || gamepad2.a) {
            robot.motorShooter.setPower(-0.87);
        }

        if (isHoldingShooterAngle) {
            if (robot.motorAngle.getCurrentPosition() >= angleShootPos) {
                // moved to pos
                robot.motorAngle.setPower(0);
            } else if (robot.motorAngle.getCurrentPosition() > angleShootPos + 300) {
                robot.motorAngle.setPower(0.2);
            }
        } else {
            robot.motorAngle.setPower(0);
        }
        telemetry.addData("Angle Pos", robot.motorAngle.getCurrentPosition());
        telemetry.addData("Angle Power", robot.motorAngle.getPower());


        if (gamepad2.dpad_left) {
            robot.motorShooter.setPower(1);
        } else if (gamepad2.dpad_right) {
            robot.motorShooter.setPower(-1);
        }
    }

          /*

          Functions

          */

    protected void moveAngleUp (double ticksToMove) {
        robot.motorAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorAngle.setPower(robot.ANGLE_AUTO_UP_SPEED);
        while (opModeIsActive() && Math.abs(robot.motorAngle.getCurrentPosition()) < Math.abs(ticksToMove)) {
            telemetry.addData("AngleMotorPos", robot.motorAngle.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.motorAngle.getCurrentPosition()));
            telemetry.update();
        }
        robot.motorArm.setPower(0);
    }

    protected void shootRings () {
        if (opModeIsActive()) {
            sleep(900);
            robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
            sleep(130);
            robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
            sleep(900);
            robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
            sleep(130);
            robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
            sleep(900);
            driveOnHeadingController(-2,0.5,0);
            robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
            sleep(130);
            robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
            sleep(900);
        }
    }

    private double inchesToTicks(double inches) {
        return inches * robot.DRIVE_MOTOR_TICKS_PER_ROTATION / (robot.WHEEL_DIAMETER * Math.PI);
    }

    private double initialHeading = 0;

    protected double getHeading() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialHeading;
    }

    protected double getHeadingDiff(double targetHeading) {
        double headingDiff = getHeading() - targetHeading;
        while (headingDiff > 180) {
            headingDiff -= 360;
        }
        while (headingDiff < -180) {
            headingDiff += 360;
        }
        return headingDiff;
    }

    protected void drive(double distance, double power) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        robot.startMove(1, 0, 0, Math.abs(power) * dir);
        while (opModeIsActive() && Math.abs(robot.motorFL.getCurrentPosition()) < encoderTicks) ;
        robot.stopMove();
    }

    protected void driveOnHeadingController(double distance, double power, double targetHeading) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        robot.startMove(1, 0, 0, Math.abs(power) * dir);
        while (opModeIsActive() && Math.abs(robot.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            robot.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.stopMove();
    }

    protected void strafeOnHeading(double distance, double power, double targetHeading) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        robot.startMove(0, 1, 0, Math.abs(power) * dir);
        while (opModeIsActive() && Math.abs(robot.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            robot.startMove(0, Math.abs(power) * dir, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.stopMove();
    }

    protected void turnToHeading(double targetHeading, double power) {
        while (opModeIsActive() && Math.abs(getHeadingDiff(targetHeading)) > 6) {
            robot.startMove(0, 0, 1, power * Math.signum(getHeadingDiff(targetHeading)));
        }
        robot.stopMove();
    }

    protected void angleHighGoalBeginningPos () {
        moveAngleUpAuto(10);
        robot.servoHolder.setPosition(robot.HOLD_HIGH_GOAL_CLOSE_POS);
        moveAngleUp(robot.ANGLE_SHOOT_HIGH_POS);
        moveAngleUpAuto(75);
    }

    protected void moveAngleUpAuto (double ticksToMove) {
        robot.motorAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && Math.abs(robot.motorAngle.getCurrentPosition()) < Math.abs(ticksToMove)) {
            if (Math.abs(robot.motorAngle.getCurrentPosition()) < Math.abs(ticksToMove - robot.ANGLE_SHOOT_HIGH_AUTO_POS)) {
                robot.motorAngle.setPower(robot.ANGLE_AUTO_UP_SPEED);
            } else {
                robot.motorAngle.setPower(robot.ANGLE_AUTO_UP_SPEED);
            }
            telemetry.addData("IntakeMotorPos", robot.motorAngle.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.motorAngle.getCurrentPosition()));
            telemetry.update();
        }
        robot.motorAngle.setPower(0);
    }

    protected void shootRing (int shootAmount) {
        if (opModeIsActive()) {
            if (shootAmount == 3) {
                robot.motorShooter.setPower(-1);

                sleep(500);
                robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
                sleep(130);
                robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
                sleep(500);
                robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
                sleep(130);
                robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
                sleep(1000);

                robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
                sleep(130);
                robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
                sleep(500);

                robot.motorShooter.setPower(0);
            } else if (shootAmount == 1) {
                robot.motorShooter.setPower(-1);
                sleep(500);
                robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
                sleep(130);
                robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
                sleep(500);
                robot.motorShooter.setPower(0);
            }
        }
    }

    protected void powerShotsSetup () {
        if (opModeIsActive()) {
            // setup
            angleHighGoalBeginningPos();
//            robot.motorShooter.setPower(0.9);
//            strafeOnHeading(5,0.3,0);
//            driveOnHeading(-5,0.3,0);
//
//            // first powershot
//            shootRing(1);
//
//            // second powershot
//            strafeOnHeading(10,0.3,0);
//            shootRing(1);
//
//            //third powershot
//            strafeOnHeading(10,0.3,0);
//            shootRing(1);

        }
    }



}



