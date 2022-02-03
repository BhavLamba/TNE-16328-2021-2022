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
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.addData("Barcode Position", barcodePosition);

        switch (barcodePosition) {
            case Left:
                break;
            case Center:
                break;
            case Right:
                break;
        }

    }

}
