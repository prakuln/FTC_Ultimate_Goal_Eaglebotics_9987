package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class VoltageSensor {

    public static com.qualcomm.robotcore.hardware.VoltageSensor voltageSensor;
    public static void init(HardwareMap hardwareMap){
    voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
    }
    public static double getVoltage(){
        return voltageSensor.getVoltage();
    }
}
