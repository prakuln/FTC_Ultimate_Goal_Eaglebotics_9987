package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Trajectories.Coordinates;

public class Joystick {
    public static void teleopControl(Gamepad gamepad1){
        Drivetrain.fieldCentricDrive(gamepad1.right_stick_x* Constants.turnPower, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

        if (gamepad1.y) Shooter.on(Constants.powerConstant);

        if (gamepad1.a) Shooter.off();

        if (gamepad1.left_bumper) Drivetrain.speedControl(0.4);

        else Drivetrain.speedControl(1);

        if(gamepad1.x){
            Robot.myLocalizer.setPoseEstimate(Coordinates.shoot);
            Robot.drive.setPoseEstimate(Coordinates.shoot);
            Shooter.shootThree();
        }
        if(gamepad1.b) Drivetrain.alignToShoot();

        if (gamepad1.right_bumper)Shooter.powerShot();

        if (gamepad1.dpad_down) Arm.moveUp(1);

        else if (gamepad1.dpad_up) Arm.moveDown(1);

        else Arm.stop();

        if (gamepad1.dpad_right) Arm.close();

        if (gamepad1.dpad_left) Arm.open();

        //experimental
        //if(gamepad1.left_stick_button) Constants.targetShooterPower-=0.05;
        //if(gamepad1.right_stick_button) Constants.targetShooterPower+=0.05;

        Intake.succIn(gamepad1.left_stick_y);
    }
    public static void testControl(Gamepad gamepad1){
        double LY = gamepad1.left_stick_y;
        double RY = gamepad1.right_stick_y;
        double LX = gamepad1.left_stick_x;
        double RX = gamepad1.right_stick_x;
        Drivetrain.leftFront.setPower(LY);
        Drivetrain.leftRear.setPower(LX);
        Drivetrain.rightFront.setPower(RY);
        Drivetrain.rightRear.setPower(RX);
        if(gamepad1.dpad_up) Arm.down();
        if(gamepad1.dpad_down )Arm.up();
        if(gamepad1.dpad_left) Arm.open();
        if(gamepad1.dpad_right )Arm.close();
        //if(gamepad1.dpad_left) Robot.MArm.setPower(1);
        //if(gamepad1.dpad_right )Robot.MIntake.setPower(1);
    }
}
