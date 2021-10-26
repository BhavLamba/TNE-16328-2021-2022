package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();
        strafeOnHeading(-30,0.5,0);
        telemetry.addData("BL pos", robot.motorBL.getCurrentPosition());
        telemetry.addData("BR pos", robot.motorBR.getCurrentPosition());
        telemetry.addData("FR pos", robot.motorFR.getCurrentPosition());
        telemetry.addData("FL pos", robot.motorFL.getCurrentPosition());
    }
}
