package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class RobotHardware {

    public BNO055IMU imu;

//    public DcMotor motorCarousel;

    public DriveTrain driveTrain;
    public Arm arm;

    public DcMotor motorIntake;
    public DcMotor motorSpinny;



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

        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);

        if (initIMU) {
            initializeIMU();
        }

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

    @Config
    public static class Arm {
        public DcMotor motorArm;
        public Servo servoArm;

        PIDController pidController;

        public static double KP = 0.0;
        public static double KI = 0.0;
        public static double KD = 0.0;
        public static int MOTOR_TOP = 446;
        public static int MOTOR_BOTTOM = 0;
        public static double SERVO_TOP = 0.1;
        public static double SERVO_BOTTOM = 0.5;


        public Arm(HardwareMap hardwareMap) {
            motorArm = hardwareMap.get(DcMotor.class, "motorArm");

            motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            servoArm = hardwareMap.get(Servo.class, "servoArm");
            servoArm.setDirection(Servo.Direction.REVERSE);
            servoArm.setPosition(SERVO_BOTTOM);

            pidController = new PIDController(KP, KI, KD, motorArm);
        }

        public void up() {
//            pidController.reference = MOTOR_TOP;
            motorArm.setTargetPosition(MOTOR_TOP);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setPower(0.2);
        }

        public void down() {
//            pidController.reference = MOTOR_BOTTOM;
            motorArm.setTargetPosition(MOTOR_BOTTOM);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setPower(0.2);

        }

        public void run(Telemetry telemetry) {
//            servoArm.setPosition(
//                (motorArm.getCurrentPosition() - MOTOR_BOTTOM)
//                * ((double) (SERVO_TOP - SERVO_BOTTOM) / (double) (MOTOR_TOP - MOTOR_BOTTOM)) + SERVO_BOTTOM
//            );
//            servoArm.setPosition(
//                    (motorArm.getTargetPosition() - motorArm.getCurrentPosition()) /  (double) MOTOR_TOP * (SERVO_TOP - SERVO_BOTTOM) + SERVO_BOTTOM
//            );
            double position = -((MOTOR_TOP - motorArm.getCurrentPosition())/ (double) MOTOR_TOP - 1) * (SERVO_TOP - SERVO_BOTTOM) + SERVO_BOTTOM;

            servoArm.setPosition(position);
//            servoArm.setPosition(Range.clip(position, SERVO_BOTTOM, SERVO_TOP));

            telemetry.addData("Servo Target", position);
        }

    }

    public class DriveTrain {
        public DcMotor motorFL;
        public DcMotor motorFR;
        public DcMotor motorBL;
        public DcMotor motorBR;
        public DcMotor[] motors;



        public DriveTrain(HardwareMap hardwareMap) {
            motorFL = hardwareMap.get(DcMotor.class, "motorFL");
            motorFR = hardwareMap.get(DcMotor.class, "motorFR");
            motorBL = hardwareMap.get(DcMotor.class, "motorBL");
            motorBR = hardwareMap.get(DcMotor.class, "motorBR");
            motors = new DcMotor[]{motorFL, motorFR, motorBL, motorBR};


            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
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
            for (DcMotor motor: motors) {
                motor.setPower(0);
            }
        }

        public void resetDriveEncoders() {
            for (DcMotor motor: motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void telemetryUpdate(Telemetry telemetry) {
            telemetry.addData("BL pos", motorBL.getCurrentPosition());
            telemetry.addData("BR pos", motorBR.getCurrentPosition());
            telemetry.addData("FR pos", motorFR.getCurrentPosition());
            telemetry.addData("FL pos", driveTrain.motorFL.getCurrentPosition());
        }
    }

    public static class Sensors {
        public DistanceSensor colorSensor;

        public Sensors(HardwareMap hardwareMap) {
            colorSensor = hardwareMap.get(DistanceSensor.class, "sensorColor");
        }

        public void telemetryUpdate(Telemetry telemetry) {
            telemetry.addData("Color Distance", colorSensor.getDistance(DistanceUnit.INCH));

        }
    }

    public class Extra {
        public DcMotor motorIntake;
        public DcMotor motorCarosel;
    }
}


