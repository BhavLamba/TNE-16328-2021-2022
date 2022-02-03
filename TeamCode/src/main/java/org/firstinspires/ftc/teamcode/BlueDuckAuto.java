package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous
public class BlueDuckAuto extends AutoCommon{

    double POWER = 0.2;

    @Override
    public void runOpMode() {
        super.runOpMode();

        robot.arm.servoArm.setPosition(robot.arm.SERVO_HOVER);
        robot.arm.up();
        sleep(5000);
        driveOnHeading(12, 0.2, 0);
        turnToHeading(30, 0.2);
        driveOnHeading(5,0.2,30);
        robot.arm.drop();
    }

}

