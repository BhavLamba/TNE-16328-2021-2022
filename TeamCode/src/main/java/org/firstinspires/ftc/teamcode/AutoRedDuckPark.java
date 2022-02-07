package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRedDuckPark extends AutoCommon {

//    BarcodePosition barcodePosition;

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Status", "Running");
        telemetry.addData("Barcode Position", barcodePosition);
        driveOnHeading(15, 0.3, 0);
        turnToHeading(90, 0.3);
        driveOnHeading(25, 0.3, 90);
    }
}
