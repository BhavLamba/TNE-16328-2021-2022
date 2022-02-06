package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoBlueDuckPark extends AutoCommon {

//    BarcodePosition barcodePosition;

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Status", "Running");
        telemetry.addData("Barcode Position", barcodePosition);
        driveOnHeading(16.5, 0.3, 0);
        sleep(500);
        turnToHeading(-90, 0.3);
        sleep(500);
        driveOnHeading(25, 0.3, -90);
    }
}
