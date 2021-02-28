package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double baseVoltage = 13.5; //EDIT THIS TO CHANGE THE BASE VOLTAGE
    public static double speed = 1; //Speed Control
    public static final double targetShooterPower = 0.45;    //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER
    public static final double targetShotPower = 0.4; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER FOR THE POWER SHOT
    public static final double turnPower = 0.7; //Turning Power
    public static final double powerConstant = baseVoltage*targetShooterPower;
    public static final double shotConstant = baseVoltage*targetShotPower;
    public static final int PWR_SHOT_LEFT_TURN = 33;
    public static final int PWR_SHOT_CENTER_TURN = 35;
    public static final int PWR_SHOT_RIGHT_TURN = 35;

}
