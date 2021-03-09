package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Mecanum_Drive.MyMecanumDrive;
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
    private VoltageSensor ExpansionHub1_VoltageSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        Robot.leftRear = hardwareMap.dcMotor.get("leftRear");
        Robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        Robot.rightRear = hardwareMap.dcMotor.get("rightRear");
        Robot.leftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        Robot.rightShooter = hardwareMap.dcMotor.get("MRightShooter");
        Robot.mIntake = hardwareMap.dcMotor.get("MIntake");
        Robot.mArm = hardwareMap.dcMotor.get("MArm");
        Robot.voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        Robot.leftClaw = hardwareMap.servo.get("LClaw");
        Robot.rightClaw = hardwareMap.servo.get("RClaw");
        Robot.cam = hardwareMap.servo.get("Cam");
        Robot.hopper = hardwareMap.servo.get("Hopper");
        Hopper.back();
        Arm.close();
        Camera.out();
        Robot.drive = new MyMecanumDrive(hardwareMap);
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
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
