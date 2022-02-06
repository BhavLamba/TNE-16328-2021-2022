package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous
public class AutoTest extends AutoCommon {

//    BarcodePosition barcodePosition;

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("AfterWaitforSignal",99);
        telemetry.update();

//            armUp(3);
//            driveOnHeading(5,0.1,0);
        driveOnHeading(15, 0.3, 0);
        sleep(1000);
        turnToHeading(90,0.3);
        sleep(1000);
        driveOnHeading(15,0.3,90);
        sleep(1000);
        turnToHeading(180,0.3);
        sleep(10000);
//            turnToHeading(90, 0.5);
//            driveOnHeading(10, 0.5, 90);
////            strafeOnHeading(20, 1, 90);
//            turnToHeading(0,0.5);
//        driveOnHeading(5,0.5,0);
//        turnToHeading(90,0.5);
//        driveOnHeading(10,0.5,90);
//        driveOnHeading(5,0.5,0);
//        turnToHeading(90,0.5);
//        driveOnHeading(10,0.5,90);
//        strafeOnHeading(20,-.5,90);
////        moveArmUp(320);
////        robot.arm.motorArm.setPower(-0.01);
//        sleep(500000);


    }
}

