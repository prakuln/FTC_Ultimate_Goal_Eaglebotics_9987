package org.firstinspires.ftc.teamcode.Robot;

public class Intake {
    public static void stop(){
        Robot.mIntake.setPower(0);
    }
    public static void succ(double value){
        Robot.mIntake.setPower(value);
    }
    public static void succIn(double value){
        succ(-value);
    }
    public static void succOut(double value){
        succ(value);
    }
}
