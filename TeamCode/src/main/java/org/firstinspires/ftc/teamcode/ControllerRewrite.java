/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


@TeleOp
public class ControllerRewrite extends OpMode
{
    RobotHardware robot;
    Toggle intake;
    ElapsedTime runtime;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap, false);
        intake = new Toggle(robot.motorIntake, 1);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        telemetry.addData("Status", "Running");


    }


    @Override
    public void loop() {
        driveControl();
        intakeControl();
        carouselControl();
        armControl();
//        servoArmControl();
//        manualArmControl();
    }


    private void driveControl() {
        double scale = 0.5;
        if (gamepad1.left_bumper) {
            gamepad1.rumble(500);
            scale = 0.8;
        } else if (gamepad1.left_trigger > 0.5) {
            scale = 0.1;
        }

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        robot.driveTrain.startMove(drive, strafe, turn, scale);

        robot.driveTrain.telemetryUpdate(telemetry);
    }

    private void intakeControl() {
        telemetry.addData("distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
        intake.monitor(gamepad1.dpad_right);
// may be moves to run off servo pos instead of color sensor, it gets wonky
        if (robot.distanceSensor.getDistance(DistanceUnit.CM) > 8.0) {
            robot.motorIntake.setPower(1);
        }
//        else if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 8.0) {
//            robot.motorIntake.setPower(0);
//        }
        else {
            if (robot.arm.currentState == RobotHardware.Arm.States.INTAKE) {
                robot.arm.hover();
            }
            robot.motorIntake.setPower(0);
        }
    }

    private void newIntakeControl() {

    }
    private void carouselControl() {
        if (gamepad1.right_trigger > 0.3) {
            robot.motorCarousel.setPower(-0.075);
        } else if (gamepad1.left_trigger > 0.3 && robot.driveTrain.motorFL.getPower() == 0) {
            robot.motorCarousel.setPower(0.1);
        } else if (gamepad1.left_trigger > 0.3 && gamepad1.right_trigger > 0.3) {
            robot.motorCarousel.setPower(-0.1);
        } else {
            robot.motorCarousel.setPower(0);
        }

//        robot.motorCarousel.setPower(-gamepad1.right_trigger);
        telemetry.addData("speed", gamepad1.right_trigger);
    }

    private void armControl() {
        if (gamepad1.y) {
            robot.arm.up();
        }
        if (gamepad1.b) {
            robot.arm.down();
        }

        if (gamepad1.right_bumper) {
            robot.arm.drop();
        }

        robot.arm.run();
        telemetry.addData("motorArm pos", robot.arm.motorArm.getCurrentPosition());
        telemetry.addData("servoArm pos", robot.arm.servoArm.getPosition());
        telemetry.addData("flicker pos", robot.arm.servoFlicker.getPosition());

        telemetry.addData("state", robot.arm.currentState);
    }

    public void manualArmControl() {
        if (gamepad2.y) {
            robot.arm.motorArm.setPower(0.1);
        } else if (gamepad2.b) {
            robot.arm.motorArm.setPower(-0.1);
        } else {
            robot.arm.motorArm.setPower(0);
        }
    }

//    private boolean prevaPress = true;
//    private boolean isHover = true;
//
//    private void servoArmControl() {
//        if (gamepad1.a && !prevaPress) {
//            isHover = !isHover;
//            robot.arm.setServoArm(isHover);
//        }
//        prevaPress = gamepad1.a;
//    }

//    private void servoArmControl() {
//        if (gamepad1.a) {
//            robot.arm.intake();
//        } else if (gamepad1.x) {
//            robot.arm.hover();
//        } else if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 8.0) {
//            robot.arm.hover();
//        }
//    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private class Toggle {
        boolean value = false;
        boolean lastPress = false;
        double power;
        DcMotor motor;

        private Toggle(DcMotor setMotor, double setPower) {
            motor = setMotor;
            power = setPower;
        }

        private void monitor(boolean input) {
            if (input && !lastPress) {
                value = !value;
            }
            lastPress = input;
            if (value) {
                motor.setPower(power);
            } else {
                motor.setPower(0);
            }
        }

        private void swap() {
            value = !value;
        }
    }

}
