package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoBlueDuck extends AutoCommon {

//    BarcodePosition barcodePosition;

    @Override
    public void runOpMode() {
        super.runOpMode();
//        strafeOnHeading(3, 0.2, 0);
        timeDriveForwardBack(1000, 0.05);
        robot.motorCarousel.setPower(-0.3);
        timeDriveForwardBack(200,0.05);
        sleep(5000);
        timeStrafe(1000,-0.2);
        timeDriveForwardBack(500,0.3);
//        driveOnHeading(15, 0.2);
    }
}
