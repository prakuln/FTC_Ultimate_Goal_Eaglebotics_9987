package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public static DcMotor mArm;
    public static Servo leftClaw;
    public static Servo rightClaw;
    public static void init(HardwareMap hardwareMap){
        mArm = hardwareMap.dcMotor.get("MArm");
        leftClaw = hardwareMap.servo.get("LClaw");
        rightClaw = hardwareMap.servo.get("RClaw");
    }
    public static void open(){
        leftClaw.setPosition(1);
        rightClaw.setPosition(1);
    }
    public static void close(){
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
    }
    public static void move(double power){
        mArm.setPower(power);
    }
    public static void moveUp(double value){
        move(-value);
    }
    public static void moveDown(double value){
        move(value);
    }
    public static void stop(){
        mArm.setPower(0);
    }
    public static void up(){
        Robot.wait(500);
        mArm.setPower(-1);
        Robot.wait(1500);
        mArm.setPower(0);
    }
    public static void down(){
        mArm.setPower(1);
        Robot.wait(1500);
        mArm.setPower(0);
    }
}
