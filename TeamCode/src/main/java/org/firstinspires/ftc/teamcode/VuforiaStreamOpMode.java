package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY =  "AWAydHD/////AAABmXGZ9mQxg09AvxhSoY5XwiUiKg1MPonVQDDS"
            + "nPNo+YPMZ8VgPFUW0TcIMXrdaUXiSIyJCwCD7AtpPBT3x0GMgxihOuroB4VTSN/eV8W8w9QmYnX2lo0VNuVFs0sQ"
            + "8Loq4jDIf2fPN0UdBHoQegRmV16sdDkYPE9tClFPxAL7oN8h82ETCyP40SZPORsbGZHRCMF5keXzhL6zoNBzD3MT"
            + "XNCTgIyoPy83Oz0RvplOH9IYrYzXemfsCv667hDX3fFkcly4W2oNfMtwf2Z1vuX1S89Mkgu0R+KEVt25PAHtLTf+"
            + "Ri2RMAEtUxW49I8Ic75dNWRno7LC1+NrXDJ5Iis693C/fUcNAC4S5uYRCmNTskz5";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}