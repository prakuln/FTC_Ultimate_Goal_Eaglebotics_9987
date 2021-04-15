package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Trajectories.Autocode;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
//@Disabled
@Autonomous(group = "OpModes")
public class Roadrunner_Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot.initTest(hardwareMap, telemetry);
        waitForStart();
        if (isStopRequested()) return;
        Autocode.test(telemetry);
    }
}
