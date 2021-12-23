package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



public class RobotHardware {

    public BNO055IMU imu;

//    public DcMotor motorCarousel;

    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    public DcMotor motorIntake;
    public DcMotor motorSpinny;

    public Servo servoRArm;
    public Servo servoLArm;
    public Servo servoFlicker;
    public Servo servoRotate;

    public static final double WHEEL_DIAMETER = 6.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 385.5; //changing from 537.6

    public static final double SERVO_RIGHT_PICKUP_POS = 0.765;
    public static final double SERVO_LEFT_PICKUP_POS = 0.115;
    public static final double SERVO_RIGHT_TOP_POS = 0.08;
    public static final double SERVO_LEFT_TOP_POS = 0.85;
    public static final double SERVO_RIGHT_INIT_POS = 0.05;
    public static final double SERVO_LEFT_INIT_POS = 0.8;
    public static final double SERVO_FLICKER_INIT_POS = 0;
    public static final double SERVO_FLCIKER_OPEN_POS = 0.14;
    public static final double SERVO_FLICKER_CLOSE_POS = 0.31;
    public static final double SERVO_ROTATE_INIT_POS = 0.49;
    public static final double SERVO_ROTATE_DODGE_MOTOR_POS = 0.13;
    public static final double SERVO_ROTATE_PICKUP_POS = 0.53;
    public static final double SERVO_ROTATE_DROP_POS = 0.6;



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
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
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
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSpinny = hardwareMap.get(DcMotor.class, "motorSpinny");
        motorSpinny.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSpinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSpinny.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSpinny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoLArm = hardwareMap.get(Servo.class, "servoLArm");
        servoRArm = hardwareMap.get(Servo.class, "servoRArm");
        servoFlicker = hardwareMap.get(Servo.class, "servoFlicker");
        servoRotate = hardwareMap.get(Servo.class, "servoRotate");
        servoLArm.setPosition(SERVO_LEFT_PICKUP_POS);
        servoRArm.setPosition(SERVO_RIGHT_TOP_POS);
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

        double maxPower = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)), Math.max(Math.abs(powerBL), Math.abs(powerBR)));
        double max = (maxPower < 1) ? 1 : maxPower;

        motorFL.setPower(Range.clip(powerFL / max, -1, 1));
        motorFR.setPower(Range.clip(powerFR / max, -1, 1));
        motorBL.setPower(Range.clip(powerBL / max, -1, 1));
        motorBR.setPower(Range.clip(powerBR / max, -1, 1));
    }



    public void stopMove() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void setDrop(boolean isClose) {
        if (isClose) {
//            servoRotate.setPosition(SERVO_ROTATE_DODGE_MOTOR_POS);
            servoFlicker.setPosition(SERVO_FLICKER_CLOSE_POS);
        } else {
//            servoRotate.setPosition(SERVO_ROTATE_DROP_POS);
            servoFlicker.setPosition(SERVO_FLCIKER_OPEN_POS);
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


