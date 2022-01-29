package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous
public class BlueDuckAuto extends AutoCommon{

    double POWER = 0.2;

    @Override
    public void init() {
        super.init();
//        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
//        robot.arm.hover();
//        robot.arm.up();
//        driveOnHeading(12, 0.2, 0);
//        turnToHeading(30, 0.2);
//        driveOnHeading(5,0.2,30);
//        robot.arm.drop();
    }

    @Override
    public void start() {
        super.start();

        robot.arm.up();
        driveOnHeading(12, 0.2, 0);
        turnToHeading(30, 0.2);
        driveOnHeading(5,0.2,30);
        robot.arm.drop();

    }

    @Override
    public void loop() {
        telemetry.addData("angle", getHeading());
//        telemetry.addData("position", robot.imu.getPosition());
    }
}
