package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;
    protected BarcodePosition barcodePosition;
    protected ElapsedTime runtime = new ElapsedTime();
//    protected boolean opMode = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap, true);
        initialHeading = getHeading();
        telemetry.update();

        initializeCamera();
        runCamera();
        runtime.reset();
        telemetry.update();

//        opMode = true;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        telemetry.update();

        telemetry.addData("Status", "Running");
        telemetry.addData("Barcode Position", barcodePosition);
        telemetry.update();


    }


    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    public static double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    public static double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 180.0, 30);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 120);
    private void initializeCamera() {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0.0, 0.0, 0.65, 0.0);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 10);

    }

    protected BarcodePosition runCamera() {
        while (!isStarted())
        {
            if(pipeline.error){
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            // testing(pipeline);

            // Watch our YouTube Tutorial for the better explanation

            double rectangleArea = pipeline.getRectArea();

            //Print out the area of the rectangle that is found.
            telemetry.addData("Rectangle Area", rectangleArea);

            //Check to see if the rectangle has a large enough area to be a marker.
            if(rectangleArea > minRectangleArea){
                //Then check the location of the rectangle to see which barcode it is in.

                double midpointX = pipeline.getRectMidpointX();
                int rectWidth = pipeline.viewRect.width;
                if(midpointX > rightBarcodeRangeBoundary * rectWidth){
                    telemetry.addData("Barcode Position", "Right");
                    barcodePosition = BarcodePosition.Right;
                }
                else if(midpointX < leftBarcodeRangeBoundary * rectWidth){
                    telemetry.addData("Barcode Position", "Left");
                    barcodePosition = BarcodePosition.Left;
                }
                else {
                    telemetry.addData("Barcode Position", "Center");
                    barcodePosition = BarcodePosition.Center;
                }
            }

            telemetry.update();
        }
        return barcodePosition;
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

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) ;
        robot.driveTrain.stopMove();
    }

    protected void driveOnHeading(double distance, double power, double targetHeading) {
        targetHeading = -targetHeading;
        double dir = Math.signum(-distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks && opModeIsActive()) {
            telemetry.addData("current heading", getHeading());
            double turnMod = getHeadingDiff(targetHeading) / 100;
            telemetry.addData("turn mod", turnMod);
            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
            telemetry.update();
        }
        robot.driveTrain.stopMove();
    }

//    protected void driveOnHeading(double distance, double power, double targetHeading) {
//        double dir = Math.signum(distance * power);
//        if (dir == 0) return;
//
//        double encoderTicks = inchesToTicks(Math.abs(distance));
//
//        robot.driveTrain.resetDriveEncoders();
//        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
//        while (opModeIsActive() && Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) {
//            double turnMod = getHeadingDiff(targetHeading) / 100;
//            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
//        }
//        robot.driveTrain.stopMove();
//    }

    protected void driveOnHeadingRamp(double driveDistance, double minPower, double maxPower, double rampDistance, double targetHeading) {
        double dir = Math.signum(driveDistance);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(driveDistance));
        double rampTicks = inchesToTicks(Math.abs(rampDistance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(minPower) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            double startRampPower = minPower + (maxPower - minPower) * (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) / rampTicks);
            double endRampPower = minPower + (maxPower - minPower) * (Math.abs(encoderTicks - robot.driveTrain.motorFL.getCurrentPosition()) / (rampTicks * 2));
            double power = Range.clip(Math.min(startRampPower, endRampPower), minPower, maxPower);
            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.driveTrain.stopMove();
    }


    protected void strafeOnHeading(double distance, double power, double targetHeading) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(0, 1, 0, Math.abs(power) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            robot.driveTrain.startMove(0, Math.abs(power) * dir, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.driveTrain.stopMove();
    }

    protected void timeDriveForwardBack(long time, double power) {
        robot.driveTrain.motorFR.setPower(power);
        robot.driveTrain.motorFL.setPower(power);
        robot.driveTrain.motorBL.setPower(power);
        robot.driveTrain.motorBR.setPower(power);
        sleep(time);
        robot.driveTrain.stopMove();
    }

    protected void timeStrafe(long time, double power)  {
        robot.driveTrain.motorFR.setPower(-power);
        robot.driveTrain.motorFL.setPower(power);
        robot.driveTrain.motorBL.setPower(-power);
        robot.driveTrain.motorBR.setPower(power);
        sleep(time);
        robot.driveTrain.stopMove();
    }

    protected void turnToHeading(double targetHeading, double power) {
        while ( Math.abs(getHeadingDiff(targetHeading)) > 6) { // added opMode=true &&
            robot.driveTrain.startMove(0, 0, 1, power * Math.signum(-getHeadingDiff(targetHeading)));
        }
        robot.driveTrain.stopMove();
    }

//    protected void turnToHeading(double targetHeading, double power) {
//        while (opModeIsActive() && Math.abs(getHeadingDiff(targetHeading)) > 6) {
//            robot.driveTrain.startMove(0, 0, 1, power * Math.signum(getHeadingDiff(targetHeading)));
//        }
//        robot.driveTrain.stopMove();
//    }

    public void armUp(int towerPos) {
        if (opModeIsActive()) {
            if (towerPos == 1) {
                robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_TOP);
                robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.motorArm.setPower(0.2);
                robot.arm.currentState = RobotHardware.Arm.States.UP;
                robot.arm.elapsedTime.reset();
            } else if (towerPos == 2) {
                robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_MID_HUB_POSITION);
                robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.motorArm.setPower(0.2);
                robot.arm.currentState = RobotHardware.Arm.States.UP;
                robot.arm.elapsedTime.reset();
            } else if (towerPos == 3) {
                robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_LOW_HUB_POSITION);
                robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.motorArm.setPower(0.2);
                robot.arm.currentState = RobotHardware.Arm.States.UP;
                robot.arm.elapsedTime.reset();
            }
        }
    }
    public void armDown() {
        robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_CLOSED);
        robot.arm.servoArm.setPosition(robot.arm.SERVO_HOVER);
        robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_BOTTOM);
        robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.motorArm.setPower(0.05);
        robot.arm.currentState = RobotHardware.Arm.States.INTAKE;
        robot.arm.elapsedTime.reset();
    }

    public enum BarcodePosition {
        Left,
        Center,
        Right
    }

    protected void moveArmDown(double ticksToMove) {
        robot.arm.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && Math.abs(robot.arm.motorArm.getCurrentPosition()) < Math.abs(ticksToMove)) {
            if (Math.abs(robot.arm.motorArm.getCurrentPosition()) < Math.abs(ticksToMove - -0.2)) {
                robot.arm.motorArm.setPower(-0.2);
            } else {
                robot.arm.motorArm.setPower(-0.2);
            }
            telemetry.addData("MotorPos", robot.arm.motorArm.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.arm.motorArm.getCurrentPosition()));
            telemetry.update();
        }
        robot.arm.motorArm.setPower(0);
    }

    protected void moveArmUp(double ticksToMove) {
        robot.arm.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.motorArm.setPower(0.3);
        while (opModeIsActive() && Math.abs(robot.arm.motorArm.getCurrentPosition()) < Math.abs(ticksToMove)) {
            telemetry.addData("MotorPos", robot.arm.motorArm.getCurrentPosition());
            telemetry.addData("TickLeft", Math.abs(ticksToMove) - Math.abs(robot.arm.motorArm.getCurrentPosition()));
            telemetry.update();
        }
        robot.arm.motorArm.setPower(0);
    }

//    public void forward(int distanceInches) {
//        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() +(int)(distanceInches*TICKS_PER_INCH));
//        frontRight.setTargetPosition(frontRight.getCurrentPosition() +(int)(distanceInches*TICKS_PER_INCH));
//        backRight.setTargetPosition(backRight.getCurrentPosition() +(int)(distanceInches*TICKS_PER_INCH));
//        backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int)(distanceInches*TICKS_PER_INCH));
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        frontRight.setVelocity(driveVelocity);
//        frontLeft.setVelocity(driveVelocity);
//        backLeft.setVelocity(driveVelocity);
//        backRight.setVelocity(driveVelocity);
//
//        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) { }
//
//        frontLeft.setVelocity(0);
//        frontRight.setVelocity(0);
//        backLeft.setVelocity(0);
//        backRight.setVelocity(0);
//    }


//    public void turnRight (int distanceInches){
//        robot.driveTrain.motorFL.setTargetPosition( robot.driveTrain.motorFL.getCurrentPosition() +(int)(distanceInches*TICKS_PER_INCH));
//        robot.driveTrain.motorFR.setTargetPosition( robot.driveTrain.motorFR.getCurrentPosition() +(int)(-distanceInches*TICKS_PER_INCH));
//        robot.driveTrain.motorBR.setTargetPosition( robot.driveTrain.motorBR.getCurrentPosition() +(int)(-distanceInches*TICKS_PER_INCH));
//        robot.driveTrain.motorBL.setTargetPosition( robot.driveTrain.motorBL.getCurrentPosition() + (int)(distanceInches*TICKS_PER_INCH));
//
//        robot.driveTrain.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.driveTrain.motorFL.setVelocity(driveVelocity);
//        robot.driveTrain.motorFR.setVelocity(driveVelocity);
//        robot.driveTrain.motorBL.setVelocity(driveVelocity);
//        robot.driveTrain.motorBR.setVelocity(driveVelocity);
//
//        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) { }
//
//        frontLeft.setVelocity(0);
//        frontRight.setVelocity(0);
//        backLeft.setVelocity(0);
//        backRight.setVelocity(0);
//
//    }

}

