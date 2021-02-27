package org.firstinspires.ftc.teamcode;

//THIS IS A BETA VERSION FOR THE TELEOP CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestHardare", group = "OpModes")
public class TestHardware extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        Robot.leftRear = hardwareMap.dcMotor.get("leftRear");
        Robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        Robot.rightRear = hardwareMap.dcMotor.get("rightRear");
        Robot.MShooter = hardwareMap.dcMotor.get("MLeftShooter");
        Robot.MIntake = hardwareMap.dcMotor.get("MIntake");
        Robot.MArm = hardwareMap.dcMotor.get("MArm");
        Robot.Cam = hardwareMap.servo.get("Cam");
        Robot.Claw = hardwareMap.servo.get("Claw");
        Robot.Stopper = hardwareMap.servo.get("Stopper");
        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {
                double LY = gamepad1.left_stick_y;
                double RY = gamepad1.right_stick_y;
                double LX = gamepad1.left_stick_x;
                double RX = gamepad1.right_stick_x;
                Robot.leftFront.setPower(LY);
                Robot.leftRear.setPower(LX);
                Robot.rightFront.setPower(RY);
                Robot.rightRear.setPower(RX);
                if(gamepad1.dpad_up) Robot.ArmDown();
                if(gamepad1.dpad_down )Robot.ArmUp();
                if(gamepad1.dpad_left) Robot.openArm();
                if(gamepad1.dpad_right )Robot.closeArm();
                //if(gamepad1.dpad_left) Robot.MArm.setPower(1);
                //if(gamepad1.dpad_right )Robot.MIntake.setPower(1);
            }
        }
    }
}
