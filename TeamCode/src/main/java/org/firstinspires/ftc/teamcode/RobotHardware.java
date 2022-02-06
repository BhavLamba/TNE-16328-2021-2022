package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.net.PortUnreachableException;


public class RobotHardware {

    public BNO055IMU imu;

//    public DcMotor motorCarousel;

    public DriveTrain driveTrain;
    public Arm arm;

    public DcMotor motorIntake;
    public DcMotor motorCarousel;

    public DistanceSensor distanceSensor;



    public static final double WHEEL_DIAMETER = 6.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 385.5; //changing from 537.6



    private HardwareMap hardwareMap;

    public RobotHardware(HardwareMap aHardwareMap, boolean initIMU) {
        hardwareMap = aHardwareMap;

        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);

        if (initIMU) {
            initializeIMU();
        }

        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorCarousel = hardwareMap.get(DcMotor.class, "motorCarousel");
        motorCarousel.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCarousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCarousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorDistance");


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
        public Servo servoFlicker;
        public States currentState = States.INTAKE;
        ElapsedTime elapsedTime;

        public static int MOTOR_TOP_HUB_POSITION = 590;
        public static int MOTOR_BOT_HUB_POSITION = 835;
        public int MOTOR_TOP = MOTOR_TOP_HUB_POSITION;
        public static int MOTOR_BOTTOM = 0;
        public static int MOTOR_MID_HUB_POSITION = 700;
        public static int MOTOR_LOW_HUB_POSITION = 750;

        public static double SERVO_TOP_HUB_POSITION = 0.85;
        public static double SERVO_BOT_HUB_POSITION = 0.05;
        public double SERVO_TOP = SERVO_TOP_HUB_POSITION;
        public static double SERVO_BOTTOM = 0.4;
        public static double SERVO_HOVER = 0.71; //.73

        public static double SERVO_DROP_OFFSET = 0.01;

        public static double FLICKER_INTAKE = 0.45; //.53
        public static double FLICKER_CLOSED = 0.56; // 0.6
        public static double FLICKER_PUSH = 0.8;
        public static double FLICKER_TOP = 0.62; //.53

        public Arm(HardwareMap hardwareMap) {
            motorArm = hardwareMap.get(DcMotor.class, "motorArm");

            motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            servoArm = hardwareMap.get(Servo.class, "servoArm");
            servoArm.setDirection(Servo.Direction.REVERSE);
            servoArm.setPosition(SERVO_HOVER);

            servoFlicker = hardwareMap.get(Servo.class, "servoFlicker");
            servoFlicker.setPosition(FLICKER_CLOSED);

            elapsedTime = new ElapsedTime();
        }

        public void up() {
            if (currentState == States.HOVER) {
                motorArm.setTargetPosition(MOTOR_TOP);
                motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorArm.setPower(0.5);
                currentState = States.UP;
                elapsedTime.reset();
            }
        }

        public void down() {
            servoArm.setPosition(SERVO_HOVER);
            servoFlicker.setPosition(FLICKER_CLOSED);
            currentState = States.HOVER;
            elapsedTime.reset();
        }

        public void intake() {
            currentState = States.INTAKE;
        }


        public void drop() {
            if (currentState == States.UP) {
                servoArm.setPosition(SERVO_TOP + SERVO_DROP_OFFSET);
                servoFlicker.setPosition(FLICKER_PUSH);
                currentState = States.DROP;
            }
        }


        public void hover() {
            if (currentState != States.HOVER) {
                elapsedTime.reset();
                servoFlicker.setPosition(FLICKER_CLOSED);
                currentState = States.HOVER;
            }
        }




        public void run() {
            if (currentState == States.HOVER && elapsedTime.seconds() >= 1) {
                motorArm.setPower(0.05);
                motorArm.setTargetPosition(MOTOR_BOTTOM);
                motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                servoArm.setPosition(SERVO_HOVER);
            } else if (currentState == States.UP && elapsedTime.seconds() >= 2) {
                motorArm.setPower(0.2);
                servoArm.setPosition(SERVO_TOP);
                servoFlicker.setPosition(FLICKER_TOP);
            } else if (currentState == States.INTAKE && elapsedTime.seconds() >= 1) {
                motorArm.setPower(0.05);
                motorArm.setTargetPosition(MOTOR_BOTTOM);
                servoArm.setPosition(SERVO_BOTTOM);
                servoFlicker.setPosition(FLICKER_INTAKE);
                motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (currentState == States.DROP && elapsedTime.seconds() >= .5) {
                currentState = States.UP;
            }
        }
//        public void run(boolean Drop) {
//            double position = ((MOTOR_TOP - motorArm.getCurrentPosition())/ (double) MOTOR_TOP - 1) * (SERVO_TOP - SERVO_BOTTOM) + SERVO_BOTTOM; // - at begging at -((motor_top
//
////            if (0 < motorArm.getCurrentPosition() && motorArm.getCurrentPosition() > 500) {
////                servoArm.setPosition(SERVO_HOVER);
////            }
//
//            if (Drop) {
//                position += SERVO_DROP_OFFSET;
//            }
//
//            servoArm.setPosition(position);
//        }

        public void setTargetLevel(Level level) {
            switch (level) {
                case TOP:
                    MOTOR_TOP = MOTOR_TOP_HUB_POSITION;
                    SERVO_TOP = SERVO_TOP_HUB_POSITION;
                    break;
                case MIDDLE:
                    MOTOR_TOP = MOTOR_BOT_HUB_POSITION;
                    SERVO_TOP = SERVO_BOT_HUB_POSITION;
                    break;
            }
        }

        enum States {
            INTAKE,
            HOVER,
            UP,
            DROP
        }

        enum Level {
            TOP,
            MIDDLE,
            BOTTOM
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

}


