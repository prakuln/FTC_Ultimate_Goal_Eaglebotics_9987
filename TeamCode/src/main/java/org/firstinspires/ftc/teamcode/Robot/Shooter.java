package org.firstinspires.ftc.teamcode.Robot;

public class Shooter {
    public static void on(double constant){
        Robot.leftShooter.setPower(constant/VoltageSensor.getVoltage());
        Robot.rightShooter.setPower(-constant/VoltageSensor.getVoltage());
    }
    public static void off(){
        Robot.leftShooter.setPower(-1);
        Robot.rightShooter.setPower(1);
        Robot.wait(500);
        Robot.leftShooter.setPower(0);
        Robot.rightShooter.setPower(0);
    }
    public static void shootOne(){ //shoot one ring
        //shooting code
        Hopper.back();
        on(Constants.powerConstant);
        Robot.wait(1000);
        //shoot the ring
        Hopper.forward();
        Robot.wait(200);
        Hopper.back(); // bring the arm back
    }
    public static void shootThree(){
        //shooting code
        Hopper.back(); //set arm back
        on(Constants.powerConstant);
        Robot.wait(500);
        for(int i=0; i<3; i++){ //shoot the rings
            Hopper.forward();
            Robot.wait(400);
            Hopper.back();
            on(Constants.powerConstant);
            Robot.wait(400);
        }
        Hopper.back(); // bring the arm back
    }
    public static void powerShot(){
        //shooting code
        Hopper.back();
        on(Constants.shotConstant);
        Robot.wait(1500);
        on(Constants.shotConstant);
        Robot.wait(500);
        Hopper.forward();
        Robot.wait(200);
        Hopper.back();
        Robot.drive.turn(Math.toRadians(Constants.powerShotTurn));
        Drivetrain.stop();
        on(Constants.shotConstant);
        Robot.wait(1000);
        Hopper.forward();
        Robot.wait(200);
        Hopper.back();
        Robot.drive.turn(Math.toRadians(-Constants.powerShotTurn*2));
        Drivetrain.stop();
        on(Constants.shotConstant);
        Robot.wait(1000);
        Hopper.forward();
        Robot.wait(200);
        Hopper.back();
        off();
    }
}
