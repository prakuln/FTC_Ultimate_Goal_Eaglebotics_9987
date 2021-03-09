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
        Robot.leftFront = hardwareMap.dcMotor.get("frontLeft");
        Robot.leftRear = hardwareMap.dcMotor.get("backLeft");
        Robot.rightFront = hardwareMap.dcMotor.get("frontRight");
        Robot.rightRear = hardwareMap.dcMotor.get("backRight");
        Robot.MShooter = hardwareMap.dcMotor.get("shooter");
        Robot.MIntake = hardwareMap.dcMotor.get("upperIntakeMotor");
        Robot.MRoller = hardwareMap.dcMotor.get("lowerIntakeMotor");
        Robot.MArm = hardwareMap.dcMotor.get("wobbleArmMotor");
        Robot.Claw = hardwareMap.servo.get("clawServo");
        Robot.Stopper = hardwareMap.servo.get("shooterStop");
        ExpansionHub1_VoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        Robot.Cam = hardwareMap.servo.get("cameraServo");
        Robot.closeArm();
        Robot.Cam.setPosition(0.9);
        Robot.drive = new MyMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        Robot.goToShoot(ExpansionHub1_VoltageSensor.getVoltage());
        Robot.wait(1000);
        Robot.ShootGoal(ExpansionHub1_VoltageSensor.getVoltage());
        Robot.shootStack(1, ExpansionHub1_VoltageSensor.getVoltage());
        Robot.wait(1000);
        Robot.ShootOne(ExpansionHub1_VoltageSensor.getVoltage());
        Robot.MIntake.setPower(0);
        //Robot.goToZone(4);
        //Robot.getWobble(4);
        //Robot.bringWobble(4);
        //Robot.goToLine(4);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
