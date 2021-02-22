package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double basevoltage = 13.5; //EDIT THIS TO CHANGE THE BASE VOLTAGE
    public static final double drivepower = 0.6; //Turning Power
    public static double speed = 1; //Speed Control
    public static final double targetshooterpower = 0.45; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER
    public static final double targetshotpower = 0.4; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER FOR THE POWER SHOT
    public static final double turnpower = 0.7; //EDIT THIS TO CHANGE THE POWER OF THE DRIVETRAIN FOR THE POWER SHOT
    public static final double powerconstant = basevoltage*targetshooterpower;
    public static final double shotconstant = basevoltage*targetshotpower;
    public static final double turnconstant = basevoltage*turnpower;
    public static final double driveconstant = basevoltage*drivepower;
    public static final int PWR_SHOT_LEFT_TURN = 33;
    public static final int PWR_SHOT_CENTER_TURN = 35;
    public static final int PWR_SHOT_RIGHT_TURN = 35;

}
