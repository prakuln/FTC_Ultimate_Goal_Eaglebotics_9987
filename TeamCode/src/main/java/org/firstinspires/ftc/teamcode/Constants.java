package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double basevoltage = 13.5; //EDIT THIS TO CHANGE THE BASE VOLTAGE
    public static double speed = 1; //Speed Control
    public static final double targetshooterpower = 0.43;    //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER
    public static final double targetshotpower = 0.38; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER FOR THE POWER SHOT
    public static final double turnpower = 0.7; //Turning Power
    public static final double powerconstant = basevoltage*targetshooterpower;
    public static final double shotconstant = basevoltage*targetshotpower;
    public static final int PWR_SHOT_LEFT_TURN = 33;
    public static final int PWR_SHOT_CENTER_TURN = 35;
    public static final int PWR_SHOT_RIGHT_TURN = 35;

}
