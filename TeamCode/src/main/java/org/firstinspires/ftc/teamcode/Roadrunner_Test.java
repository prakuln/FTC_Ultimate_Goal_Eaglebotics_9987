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
//@Disabled
@Autonomous(group = "OpModes")
public class Roadrunner_Test extends LinearOpMode {
    private VoltageSensor ExpansionHub1_VoltageSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        Robot.leftRear = hardwareMap.dcMotor.get("leftRear");
        Robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        Robot.rightRear = hardwareMap.dcMotor.get("rightRear");
        Robot.MLeftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        Robot.MRightShooter = hardwareMap.dcMotor.get("MRightShooter");
        Robot.MIntake = hardwareMap.dcMotor.get("MIntake");
        Robot.MArm = hardwareMap.dcMotor.get("MArm");
        Robot.ExpansionHub1_VoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        Robot.LClaw = hardwareMap.servo.get("LClaw");
        Robot.RClaw = hardwareMap.servo.get("RClaw");
        Robot.Cam = hardwareMap.servo.get("Cam");
        Robot.Hopper = hardwareMap.servo.get("Hopper");
        Robot.hopperBack();
        Robot.closeArm();
        Robot.cameraOut();
        Robot.drive = new MyMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        Robot.goToShoot();
        //Robot.shootStack(1, ExpansionHub1_VoltageSensor.getVoltage());
        //Robot.wait(1000);
        //Robot.ShootOne(ExpansionHub1_VoltageSensor.getVoltage());
        //Robot.MIntake.setPower(0);
        //Robot.goToZone(0);
        //Robot.getWobble(0);
        //Robot.bringWobble(0);
        //Robot.goToLine(0);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
