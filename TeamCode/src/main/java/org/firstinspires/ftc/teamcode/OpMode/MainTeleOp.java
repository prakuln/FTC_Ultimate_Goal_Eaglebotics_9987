package org.firstinspires.ftc.teamcode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanum_Drive.MyMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Camera;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Hopper;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Shooter;

@TeleOp(name = "MainTeleOp", group = "OpModes")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        Robot.leftRear = hardwareMap.dcMotor.get("leftRear");
        Robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        Robot.rightRear = hardwareMap.dcMotor.get("rightRear");
        Robot.leftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        Robot.rightShooter = hardwareMap.dcMotor.get("MRightShooter");
        Robot.mIntake = hardwareMap.dcMotor.get("MIntake");
        Robot.mArm = hardwareMap.dcMotor.get("MArm");
        Robot.voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        Robot.leftClaw = hardwareMap.servo.get("LClaw");
        Robot.rightClaw = hardwareMap.servo.get("RClaw");
        Robot.cam = hardwareMap.servo.get("Cam");
        Robot.hopper = hardwareMap.servo.get("Hopper");
        Robot.drive = new MyMecanumDrive(hardwareMap);
        Camera.in();
        Hopper.back();
        Arm.open();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
                Hopper.back();
                Drivetrain.fieldCentricDrive(gamepad1.right_stick_x* Constants.turnPower, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
                if (gamepad1.y) {
                    Shooter.on(Constants.powerConstant);
                }
                if (gamepad1.a) {
                    Shooter.off();
                }
                if (gamepad1.left_bumper){ //code for speed change
                    Drivetrain.speedControl(0.4);
                }
                else{
                    Drivetrain.speedControl(1);
                }
                if (gamepad1.x){
                    Shooter.shootThree();

                }
                if (gamepad1.b){
                    Shooter.shootOne();

                }
                if (gamepad1.right_bumper){ //code for shooting the power shots
                    Shooter.powerShot();
                }
                if (gamepad1.dpad_down){ // code for the arm
                    Arm.moveUp(1);
                }
                else if (gamepad1.dpad_up){
                    Arm.moveDown(1);
                }
                else{
                    Arm.stop();
                }
                if (gamepad1.dpad_right){
                    Arm.close();
                }
                if (gamepad1.dpad_left){
                    Arm.open();
                }
                Intake.succIn(gamepad1.left_stick_y);

            }

        }

    }


}