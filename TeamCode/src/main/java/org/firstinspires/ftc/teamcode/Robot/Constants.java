package org.firstinspires.ftc.teamcode.Robot;

public class Constants {
    public static final double baseVoltage = 13.5; //EDIT THIS TO CHANGE THE BASE VOLTAGE
    public static double speed = 1; //Speed Control
    public static double targetShooterPower = 0.44;    //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER
    public static final double targetShotPower = 0.36; //EDIT THIS TO CHANGE THE POWER OF THE SHOOTER FOR THE POWER SHOT
    public static final double turnPower = 0.7; //Turning Power
    public static final double powerConstant = baseVoltage*targetShooterPower;
    public static final double shotConstant = baseVoltage*targetShotPower;
    public static final double powerShotTurn = 5.5;

}
