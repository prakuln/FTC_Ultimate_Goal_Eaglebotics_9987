package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static DcMotor mIntake;
    public static void init(HardwareMap hardwareMap){
        mIntake = hardwareMap.dcMotor.get("MIntake");
    }
    public static void stop(){
        mIntake.setPower(0);
    }
    public static void succ(double value){
        mIntake.setPower(value);
    }
    public static void succIn(double value){
        succ(-value);
    }
    /*public static void succOut(double value){
        succ(value);
    }*/
}
