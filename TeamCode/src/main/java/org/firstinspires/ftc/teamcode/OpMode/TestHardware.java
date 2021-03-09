package org.firstinspires.ftc.teamcode.OpMode;

//THIS IS A BETA VERSION FOR THE TELEOP CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Disabled
@TeleOp(name = "TestHardware", group = "OpModes")
public class TestHardware extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {
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
    }
}
