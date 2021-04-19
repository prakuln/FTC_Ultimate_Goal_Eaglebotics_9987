package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hopper {
    public static Servo hopper;

    public static void init(HardwareMap hardwareMap){
        hopper = hardwareMap.servo.get("Hopper");
    }
    public static void back(){
        hopper.setPosition(0.95);
    }
    public static void forward(){
        hopper.setPosition(0.81);
    }
}
