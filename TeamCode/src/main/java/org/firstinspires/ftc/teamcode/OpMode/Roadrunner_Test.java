package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Hopper;
import org.firstinspires.ftc.teamcode.Robot.Navigation;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
//@Disabled
@Autonomous(group = "OpModes")
public class Roadrunner_Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap);
        Hopper.back();
        Arm.close();
        Camera.out();
        waitForStart();
        if (isStopRequested()) return;
        Navigation.goToShoot();
        //Robot.shootStack(1, ExpansionHub1_VoltageSensor.getVoltage());
        //Robot.wait(1000);
        //Robot.ShootOne(ExpansionHub1_VoltageSensor.getVoltage());
        //Robot.MIntake.setPower(0);
        //Robot.goToZone(0);
        //Robot.getWobble(0);
        //Robot.bringWobble(0);
        //Robot.goToLine(0);
        Drivetrain.setEndPose();
        while (!isStopRequested() && opModeIsActive()) {
            Robot.wait(1);
        }
    }
}
