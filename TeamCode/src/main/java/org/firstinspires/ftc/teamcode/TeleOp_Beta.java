package org.firstinspires.ftc.teamcode;

//THIS IS A BETA VERSION FOR THE TELEOP CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech_drive.MyMecanumDrive;
import org.firstinspires.ftc.teamcode.mech_drive.StandardTrackingWheelLocalizer;

@TeleOp(name = "TeleOp_Beta", group = "OpModes")
public class TeleOp_Beta extends LinearOpMode {
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
        Robot.leftClaw = hardwareMap.servo.get("LClaw");
        Robot.rightClaw = hardwareMap.servo.get("RClaw");
        Robot.cam = hardwareMap.servo.get("Cam");
        Robot.hopper = hardwareMap.servo.get("Hopper");
        Robot.voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        Robot.drive = new MyMecanumDrive(hardwareMap);
        Robot.myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        Robot.drive.setPoseEstimate(Coordinates.end);
        Robot.myLocalizer.setPoseEstimate(Coordinates.end);
        Robot.cameraIn();
        Robot.hopperBack();
        Robot.openArm();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
                Robot.drive.update();
                Robot.myLocalizer.update();
                Robot.hopperBack();
                Pose2d myPose = Robot.myLocalizer.getPoseEstimate();

                telemetry.addData("x", myPose.getX());
                telemetry.addData("y", myPose.getY());
                telemetry.addData("heading", myPose.getHeading());
                Robot.fieldCentricDrive(gamepad1.right_stick_x*Constants.turnPower, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
                if (gamepad1.y) {
                    Robot.shooterOn(Constants.powerConstant);
                }
                if (gamepad1.a) {
                    Robot.shooterOff();
                }
                if (gamepad1.left_bumper){ //code for speed change
                   Robot.SpeedControl(0.4);
                }
                else{
                    Robot.SpeedControl(1);
                }
                /*if (gamepad1.x){
                    Robot.alignStraight();
                    Robot.ShootGoal();
                }*/
                if(gamepad1.x){
                    telemetry.addData("Aligning to shoot", "");
                    Robot.myLocalizer.setPoseEstimate(Coordinates.shoot);
                    Robot.ShootGoal();
                }
                if(gamepad1.b){
                    telemetry.addData("Resetting odometry position", "");
                    Robot.alignToShoot();

                }
                /*if (gamepad1.b){
                    Robot.alignStraight();
                    Robot.ShootOne();
                }*/
                if (gamepad1.right_bumper){ //code for shooting the power shots
                    Robot.PowerShot();
                }
                if (gamepad1.dpad_down){ // code for the arm
                    Robot.mArm.setPower(-1);
                }
                else if (gamepad1.dpad_up){
                    Robot.mArm.setPower(1);
                }
                else{
                    Robot.mArm.setPower(0);
                }
                if (gamepad1.dpad_right){
                    Robot.closeArm();
                }
                if (gamepad1.dpad_left){
                    Robot.openArm();
                }
                Robot.mIntake.setPower(-gamepad1.left_stick_y);

            }

        }

    }


}