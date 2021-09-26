package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap, true);
        initialHeading = getHeading();


        telemetry.addData("Status", "Waiting for Vuforia...");
        telemetry.update();
        //initVuforia();

        initOpenCVCamera ();

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
            double endRampPower = minPower + (maxPower - minPower) * (Math.abs(encoderTicks - robot.motorFL.getCurrentPosition()) / (rampTicks*2));
            double power = Range.clip(Math.min(startRampPower, endRampPower), minPower, maxPower);
            robot.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.stopMove();
    }

    protected void driveOnHeadingIntake (double driveDistance, double power, double targetHeading, double IntakeMs) {

        double dir = Math.signum(driveDistance);
        if (dir == 0) return;

        boolean isIntakeMoving = false;
        boolean isDoneDriving = false;
        ElapsedTime IntakeTime = new ElapsedTime();

        double encoderTicks = inchesToTicks(Math.abs(driveDistance));

        robot.resetDriveEncoders();
        robot.startMove(1, 0, 0, Math.abs(power) * dir);

        while (opModeIsActive() && ((Math.abs(robot.motorFL.getCurrentPosition()) < encoderTicks) || IntakeTime.milliseconds() < IntakeMs)) {

            double turnMod = getHeadingDiff(targetHeading) / 100;


            if (!isDoneDriving) {
                robot.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
            }
            else
                robot.stopMove();

            if (isIntakeMoving) {
                robot.motorIntake.setPower(0.0);
            }
            else {
                robot.motorIntake.setPower(1.0);
            }

            if (Math.abs(robot.motorFL.getCurrentPosition()) >= encoderTicks)
                isDoneDriving = true;

            if (IntakeTime.milliseconds() >= IntakeMs)
                isIntakeMoving = true;
        }
        robot.stopMove();
    }

    protected void driveOnHeadingArm (double driveDistance, double power, double targetHeading, double maxArmTicks) {

        double dir = Math.signum(driveDistance);
        if (dir == 0) return;

        boolean isArmMoving = false;
        boolean isDoneDriving = false;

        double encoderTicks = inchesToTicks(Math.abs(driveDistance));

        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetDriveEncoders();
        robot.startMove(1, 0, 0, Math.abs(power) * dir);

        while (opModeIsActive() && ((Math.abs(robot.motorFL.getCurrentPosition()) < encoderTicks) || Math.abs(robot.motorArm.getCurrentPosition()) < maxArmTicks)) {

            double turnMod = getHeadingDiff(targetHeading) / 100;


            if (!isDoneDriving) {
                robot.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
            }
            else
                robot.stopMove();

            if (isArmMoving) {
                robot.motorArm.setPower(0.0);
            }
            else {
                robot.motorArm.setPower(-1.0);
            }

            if (Math.abs(robot.motorFL.getCurrentPosition()) >= encoderTicks)
                isDoneDriving = true;

            if (Math.abs(robot.motorArm.getCurrentPosition()) >= maxArmTicks)
                isArmMoving = true;
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
        protected void moveShooter(double ticksToMove) {
                robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (opModeIsActive() && Math.abs(robot.motorAngle.getCurrentPosition()) < Math.abs(ticksToMove)) {
                    if (Math.abs(robot.motorShooter.getCurrentPosition()) < Math.abs(ticksToMove - robot.SHOOTER_AUTO_SPEED)) {
                        robot.motorShooter.setPower(robot.SHOOTER_AUTO_SPEED);
                    } else {
                        robot.motorShooter.setPower(robot.SHOOTER_AUTO_SPEED);
                    }
                    telemetry.addData("ShooterMotorPos", robot.motorShooter.getCurrentPosition());
                    telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.motorShooter.getCurrentPosition()));
                    telemetry.update();
                }
                robot.motorAngle.setPower(0);
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

    protected void moveAngleDown1 (double ticksToMove) {
        robot.motorAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && Math.abs(robot.motorAngle.getCurrentPosition()) < Math.abs(ticksToMove)) {
            if (Math.abs(robot.motorAngle.getCurrentPosition()) < Math.abs(ticksToMove - robot.ANGLE_SHOOT_HIGH_POS)) {
                robot.motorAngle.setPower(robot.ANGLE_AUTO_DOWN_SPEED);
            } else {
                robot.motorAngle.setPower(robot.ANGLE_AUTO_DOWN_SPEED);
            }
            telemetry.addData("AngleMotorPos", robot.motorAngle.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.motorAngle.getCurrentPosition()));
            telemetry.update();
        }
        robot.motorAngle.setPower(0);
    }

    protected void intakeMoving(double ticksTomove) {
        robot.motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorIntake.setPower(robot.INTAKE_AUTO_SPEED);
        while  (opModeIsActive() && Math.abs(robot.motorIntake.getCurrentPosition()) < Math.abs(ticksTomove)) {
            telemetry.addData("IntakeMotorPos", robot.motorIntake.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksTomove) - Math.abs(robot.motorIntake.getCurrentPosition()));
            telemetry.update();
        }
        robot.motorIntake.setPower(0);
    }


    protected void moveArmUp(double ticksToMove) {
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorArm.setPower(robot.ARM_AUTO_UP_SPEED);
        while (opModeIsActive() && Math.abs(robot.motorArm.getCurrentPosition()) < Math.abs(ticksToMove)) {
            telemetry.addData("ArmMotorPos", robot.motorArm.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.motorArm.getCurrentPosition()));
            telemetry.update();
        }
        robot.motorArm.setPower(0);
    }

    protected void moveArmDown(double ticksToMove) {
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && Math.abs(robot.motorArm.getCurrentPosition()) < Math.abs(ticksToMove)) {
            if (Math.abs(robot.motorArm.getCurrentPosition()) < Math.abs(ticksToMove - robot.ARM_AUTO_DOWN_SLOW_TICKS)) {
                robot.motorArm.setPower(robot.ARM_AUTO_DOWN_SPEED_FAST);
            } else {
                robot.motorArm.setPower(robot.ARM_AUTO_DOWN_SPEED_SLOW);
            }
            telemetry.addData("ArmMotorPos", robot.motorArm.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.motorArm.getCurrentPosition()));
            telemetry.update();
        }
        robot.motorArm.setPower(0);
    }

    protected void moveArmMiliseconds (double armPower, int msec)
        throws InterruptedException {
            if (opModeIsActive()) {
                robot.motorArm.setPower(armPower);
                sleep(msec);
                robot.motorArm.setPower(0);
            }
    }

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
    protected void shootRings (int shootAmount) {
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

    private boolean holdShooterToPosTick(double angleTicks) {
        if (Math.abs(robot.motorAngle.getCurrentPosition()) < angleTicks) {
            robot.motorAngle.setPower(0.1);
            return false;
        } else if (Math.abs(robot.motorAngle.getCurrentPosition()) >= angleTicks + 5) {
            robot.motorAngle.setPower(0);
            return false;
        } else {
            robot.motorAngle.setPower(0.001);
            return true;
        }
    }

    protected void angleHighGoalPos () {
        moveAngleUp(10);
        robot.servoHolder.setPosition(robot.HOLD_HIGH_GOAL_POS);
        moveAngleUpAuto(robot.ANGLE_SHOOT_HIGH_POS);
        moveAngleDown1(75);
    }

    protected void angleHighGoalBeginningPos () {
        moveAngleUpAuto(10);
        robot.servoHolder.setPosition(robot.HOLD_HIGH_GOAL_CLOSE_POS);
        moveAngleUp(robot.ANGLE_SHOOT_HIGH_POS);
        moveAngleUpAuto(75);
    }

    protected void moveAngleDown () {
        robot.motorAngle.setPower(-0.1);
        robot.servoHolder.setPosition(robot.HOLD_ERECT_POS);
        sleep(300);
        robot.motorAngle.setPower(0);
    }

    protected void initOpenCVCamera ()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        robot.pipeline = new SkystoneDeterminationPipeline();
        robot.webcam.setPipeline(robot.pipeline);

        robot.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.webcam.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });
    }

    public RingPosition WaitForStartRingCount() {

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        robot.webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        robot.pipeline = new SkystoneDeterminationPipeline();
//        robot.webcam.setPipeline(robot.pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        robot.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                robot.webcam.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
//            }
//        });

        //sleep(3500);
//        while (opModeIsActive() && robot.pipeline.hasReadImage) {
//            telemetry.addData("Analysis", robot.pipeline.getAnalysis());
//            telemetry.addData("Position", robot.pipeline.position);
//            telemetry.update();
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }
       // waitForStart();
        telemetry.addData("Analysis", robot.pipeline.getAnalysis());
            telemetry.addData("Position", robot.pipeline.position);
            telemetry.update();

        return robot.pipeline.position;
    }

    public enum RingPosition {
        FOUR,
        ONE,
        NONE
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {

        boolean hasReadImage = true;

        /*
         * An enum to define the skystone position
         */

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */

        // top left point of box
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(20, 180);

        // width and height
        static final int REGION_WIDTH = 130;
        static final int REGION_HEIGHT = 160;

        // pixel heights for one and four rings
        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }


        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int) Core.mean(region1_Cb).val[0];


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }
            hasReadImage = true;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }

        public RingPosition getPosition() {

            return position;
        }
    }

}


