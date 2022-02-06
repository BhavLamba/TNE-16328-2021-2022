package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoCamRedWarehouse extends AutoCommon {

//    BarcodePosition barcodePosition;

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Status", "Running");
        telemetry.addData("Barcode Position", barcodePosition);
        while (opModeIsActive()) {

            telemetry.addData("AfterWaitforSignal",99);
            telemetry.update();

//            runCamera();
            driveOnHeading(12,0.3,0);
            armUp(1);
            sleep(500);
            turnToHeading(40,0.3);
            sleep(500);
            driveOnHeading(12,0.3,40);
            sleep(500);
            robot.arm.servoArm.setPosition(robot.arm.SERVO_TOP);
            robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_PUSH);
            armDown();
            driveOnHeading(-7,0.3,40);
            sleep(500);
            turnToHeading(90,0.3);
            sleep(500);
            driveOnHeading(-15,1,90);

//            if (barcodePosition == BarcodePosition.Left) {
//
//            } else if (barcodePosition == BarcodePosition.Center) {
//                robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_CLOSED);
//                    robot.arm.servoArm.setPosition(robot.arm.SERVO_HOVER);
////                armUp(2);
//                    driveOnHeading(7, 0.3, 0);
//                    turnToHeading(30, 0.3);
//                    sleep(3000);
////                robot.arm.servoArm.setPosition(robot.arm.SERVO_TOP);
////                    driveOnHeading(8, 0.3, 30);
//                strafeOnHeading(8,1,30);
////                robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_PUSH);
//                    sleep(500);
////                armDown();
//                    sleep(5000);
//            } else if (barcodePosition == BarcodePosition.Right) {
//
//            }
//            switch (barcodePosition) {
//                case Left:
//
//
//                    break;
//                case Center:
//                    robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_CLOSED);
//                    robot.arm.servoArm.setPosition(robot.arm.SERVO_HOVER);
////                armUp(2);
//                    driveOnHeading(7, 0.3, 0);
//                    turnToHeading(30, 0.3);
//                    sleep(3000);
////                robot.arm.servoArm.setPosition(robot.arm.SERVO_TOP);
//                    driveOnHeading(8, 0.3, 30);
////                robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_PUSH);
//                    sleep(500);
////                armDown();
//                    sleep(5000);
//                    break;
//                case Right:
//                    robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_CLOSED);
//                    robot.arm.servoArm.setPosition(robot.arm.SERVO_HOVER);
//
//                    break;
//            }

        }

    }
}
