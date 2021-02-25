package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.mech_drive.MyMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "OpModes")
public class Roadrunner_Test extends LinearOpMode {
    private VoltageSensor ExpansionHub1_VoltageSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHub1_VoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 1");
        Robot.MLeftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        Robot.MRightShooter = hardwareMap.dcMotor.get("MRightShooter");
        Robot.Hopper = hardwareMap.servo.get("Hopper");
        Robot.drive = new MyMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        Robot.goToShoot(ExpansionHub1_VoltageSensor.getVoltage());
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
