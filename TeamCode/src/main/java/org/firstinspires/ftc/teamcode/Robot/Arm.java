package org.firstinspires.ftc.teamcode.Robot;

public class Arm {
    public static void open(){
        Robot.leftClaw.setPosition(1);
        Robot.rightClaw.setPosition(1);
    }
    public static void close(){
        Robot.leftClaw.setPosition(0);
        Robot.rightClaw.setPosition(0);
    }
    public static void move(double power){
        Robot.mArm.setPower(power);
    }
    public static void moveUp(double value){
        move(-value);
    }
    public static void moveDown(double value){
        move(value);
    }
    public static void stop(){
        Robot.mArm.setPower(0);
    }
    public static void up(){
        Robot.wait(500);
        Robot.mArm.setPower(-1);
        Robot.wait(1500);
        Robot.mArm.setPower(0);
    }
    public static void down(){
        Robot.mArm.setPower(1);
        Robot.wait(1500);
        Robot.mArm.setPower(0);
    }
}
