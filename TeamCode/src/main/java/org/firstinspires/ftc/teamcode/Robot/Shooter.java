package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public static DcMotor leftShooter;
    public static DcMotor rightShooter;
    public static void init(HardwareMap hardwareMap){
        leftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        rightShooter = hardwareMap.dcMotor.get("MRightShooter");
    }
    public static void on(double constant){
        leftShooter.setPower(constant/VoltageSensor.getVoltage());
        rightShooter.setPower(-constant/VoltageSensor.getVoltage());
    }
    public static void off(){
        leftShooter.setPower(-1);
        rightShooter.setPower(1);
        Robot.wait(500);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public static void shootOne(){ //shoot one ring
        //shooting code
        Hopper.back();
        on(Constants.powerConstant);
        Robot.wait(1000);
        //shoot the ring
        Hopper.forward();
        Robot.wait(200);
        Hopper.back(); // bring the arm back
    }
    public static void shootThree(){
        //shooting code
        Hopper.back(); //set arm back
        on(Constants.powerConstant);
        Robot.wait(500);
        Intake.succIn(0.5);
        for(int i=0; i<3; i++){ //shoot the rings
            Hopper.forward();
            Robot.wait(500);
            Hopper.back();
            on(Constants.powerConstant);
            Robot.wait(500);
        }
        Hopper.back(); // bring the arm back
        Intake.stop();
    }
    public static void powerShot(){
        //shooting code
        Hopper.back();
        Intake.succIn(0.5);
        on(Constants.shotConstant);
        Robot.wait(2000);
        Hopper.forward();
        Robot.wait(200);
        Hopper.back();
        Robot.drive.turn(Math.toRadians(Constants.powerShotTurn));
        Drivetrain.stop();
        on(Constants.shotConstant);
        Robot.wait(1000);
        Hopper.forward();
        Robot.wait(200);
        Hopper.back();
        Robot.drive.turn(Math.toRadians(-Constants.powerShotTurn*2));
        Drivetrain.stop();
        on(Constants.shotConstant);
        Robot.wait(1000);
        Hopper.forward();
        Robot.wait(200);
        Hopper.back();
        Intake.stop();
        off();
    }
}
