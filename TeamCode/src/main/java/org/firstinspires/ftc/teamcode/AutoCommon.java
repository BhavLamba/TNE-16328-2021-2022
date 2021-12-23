package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap, true);
        initialHeading = getHeading();


        telemetry.addData("Status", "Waiting for Vuforia...");
        telemetry.update();
        //initVuforia();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
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

    protected void driveOnHeading(double distance, double power, double targetHeading) {
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

    protected void driveOnHeadingRamp(double driveDistance, double minPower, double maxPower, double rampDistance, double targetHeading) {
        double dir = Math.signum(driveDistance);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(driveDistance));
        double rampTicks = inchesToTicks(Math.abs(rampDistance));

        robot.resetDriveEncoders();
        robot.startMove(1, 0, 0, Math.abs(minPower) * dir);
        while (opModeIsActive() && Math.abs(robot.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            double startRampPower = minPower + (maxPower - minPower) * (Math.abs(robot.motorFL.getCurrentPosition()) / rampTicks);
            double endRampPower = minPower + (maxPower - minPower) * (Math.abs(encoderTicks - robot.motorFL.getCurrentPosition()) / (rampTicks * 2));
            double power = Range.clip(Math.min(startRampPower, endRampPower), minPower, maxPower);
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
/*
    protected void turnToHeading(double targetHeading) {
        while (opModeIsActive() && Math.abs(getHeadingDiff(targetHeading)) > 6) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            robot.startMove(0, 0, Range.clip(turnMod, -1.0, 1.0));
        }
        robot.stopMove();
    }
/*
    protected void moveShooter (double shooterMove) {
        boolean isDoneShooting = false;
        double encoderTicks = inchesToTicks(Math.abs(shooterMove));
        robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
        robot.servoRingPush.setPosition(robot.PUSH_IN_POS);

        while (!isDoneShooting) {
            if (Math.abs(robot.motorShooter.getCurrentPosition()) >= Math.abs(robot.SHOOTER_AUTO_DONE_SHOOT_TICKS)) {
                isDoneShooting = true;
            }
            if (isDoneShooting) {
                robot.motorShooter.setPower(0);
            }
            if (Math.abs(robot.motorShooter.getCurrentPosition()) >= encoderTicks)
                isDoneShooting = true;
        }
    }

   */











    /*
    protected void shootRings(int shootAmount) {

        robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetDriveEncoders();

        // start the shooter motor pre-emtively
        robot.motorShooter.setPower(robot.SHOOTER_AUTO_SPEED);

            sleep(2000);

            if (opModeIsActive()) {
                // shoot the rings
                for (int i = 0; opModeIsActive() && i < shootAmount; i++) {
                    // swing the shooter arm out to fire
                    robot.servoRingPush.setPosition(robot.PUSH_OUT_POS);
                    ElapsedTime elapsedTime = new ElapsedTime();

                        sleep(130);

                    // bring the shooter arm back
                    robot.servoRingPush.setPosition(robot.PUSH_IN_POS);
                    if ((i + 1 )< shootAmount) {
                        // only wait for it to retract if we are shooting again
                        elapsedTime.reset();

                        sleep(500);
                    }
                }
            }
        // stop the shooter motor
        robot.motorShooter.setPower(0);
        robot.motorAngle.setPower(0);
    }
*/
}
