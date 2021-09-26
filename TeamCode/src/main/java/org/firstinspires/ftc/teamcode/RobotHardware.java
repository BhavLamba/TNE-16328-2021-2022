package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.easyopencv.OpenCvInternalCamera;


public class RobotHardware {

    public BNO055IMU imu;

    OpenCvInternalCamera webcam;
    AutoCommon.SkystoneDeterminationPipeline pipeline;

    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    public DcMotor motorIntake;
    public DcMotor motorShooter;
    public DcMotor motorAngle;
    public DcMotor motorArm;

    public Servo servoRingPush;
    public Servo servoClamp;
    public Servo servoHolder;


    public static final double SHOOTER_AUTO_SPEED = -1;
    public static final double SHOOTER_AUTO_DONE_SHOOT_TICKS = 800;

    public static final double INTAKE_ROTATE = 5000;
    public static final double INTAKE_AUTO_SPEED = 1;
    public static final double INTAKE_ROTATION_AUTO_MS = 3000;

    public static final double WHEEL_DIAMETER = 4.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 537.6;

    public static final double ANGLE_AUTO_UP_SPEED = -0.2;
    public static final double ANGLE_AUTO_DOWN_SPEED = 0.3;
    public static final double ANGLE_SHOOT_HIGH_POS = 250;
    public static final double ANGLE_SHOOT_POWER_POS = 150;
    public static final double ANGLE_SHOOT_HIGH_AUTO_POS = 220;


    public static final double ARM_AUTO_DOWN_SPEED_SLOW = -0.5;
    public static final double ARM_AUTO_DOWN_SPEED_FAST = -1.0;
    public static final double ARM_AUTO_DOWN_SLOW_TICKS = 500;
    public static final double ARM_AUTO_UP_SPEED = 0.5;

    public static final double CLAMP_CLOSE_POS = 0.52;
    public static final double CLAMP_OPEN_POS = 0;

    public static final double PUSH_IN_POS = 0.79;
    public static final double PUSH_OUT_POS = 0.6;
    public static final double PUSH_MOVE_IN_TIME_MS = 900;
    public static final double PUSH_MOVE_OUT_TIME_MS = 130;

    public static final double HOLD_HIGH_GOAL_POS = 0.96;
    public static final double HOLD_HIGH_GOAL_CLOSE_POS = 0.92;
    public static final double HOLD_ERECT_POS = 0.48;
    public static final double HOLD_INIT_POS = 0.79;

    private HardwareMap hardwareMap = null;

    public RobotHardware(HardwareMap aHardwareMap, boolean initIMU) {
        hardwareMap = aHardwareMap;

        if (initIMU) {
            initializeIMU();
        }

        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");

        motorShooter = hardwareMap.get(DcMotor.class, "motorShooter");
        motorShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorAngle = hardwareMap.get(DcMotor.class, "motorAngle");
//        motorAngle.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoClamp = hardwareMap.get(Servo.class, "servoClamp");
        servoRingPush = hardwareMap.get(Servo.class, "servoPush");
        servoHolder = hardwareMap.get(Servo.class, "servoHolder");
        servoClamp.setPosition(CLAMP_CLOSE_POS);
        servoRingPush.setPosition(PUSH_IN_POS);
        servoHolder.setPosition(HOLD_INIT_POS);

    }

    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitImuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        //parameters.loggingTag          = "IMU";
        //parameters.mode                = BNO055IMU.SensorMode.NDOF;

        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void startMove(double drive, double strafe, double turn, double scale) {
        double powerFL = (drive + strafe + turn) * scale;
        double powerFR = (drive - strafe - turn) * scale;
        double powerBL = (drive - strafe + turn) * scale;
        double powerBR = (drive + strafe - turn) * scale;

        double maxPower = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)), Math.max(Math.abs(powerBL), Math.abs(powerBR))); // ?? Why are we using MAX function here.
        double max = (maxPower < 1) ? 1 : maxPower;

        motorFL.setPower(Range.clip(powerFL / max, -1, 1));  // What does this clipping doing?
        motorFR.setPower(Range.clip(powerFR / max, -1, 1));
        motorBL.setPower(Range.clip(powerBL / max, -1, 1));
        motorBR.setPower(Range.clip(powerBR / max, -1, 1));
    }

    public void startMove(double drive, double strafe, double turn) {
        startMove(drive, strafe, turn, 1);
    }

    public void stopMove() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    public void setClaw(boolean isClose) {
        if (isClose) {
            servoClamp.setPosition(CLAMP_CLOSE_POS);
        } else {
            servoClamp.setPosition(CLAMP_OPEN_POS);
        }
    }

    public void resetDriveEncoders() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


