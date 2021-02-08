package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double basevoltage = 13.5; //EDIT THIS TO CHANGE THE BASE VOLTAGE
    public static final double drivepower = 0.6;
    public static final double targetshooterpower = 0.44; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER
    public static final double targetshotpower = 0.4; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER FOR THE POWER SHOT
    public static final double turnpower = 0.5; //EDIT THIS TO CHANGE THE POWER OF THE DRIVETRAIN FOR THE POWER SHOT
    public static final double powerconstant = basevoltage*targetshooterpower;
    public static final double shotconstant = basevoltage*targetshotpower;
    public static final double turnconstant = basevoltage*turnpower;
    public static final double driveconstant = basevoltage*drivepower;

}
