package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
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
        Robot.cameraIn();
        Robot.hopperBack();
        Robot.openArm();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
                Robot.hopper.setPosition(0.94);
                Robot.MechanumDriveControl(gamepad1.right_stick_x*Constants.turnPower, gamepad1.right_stick_y,  gamepad1.left_trigger, gamepad1.right_trigger);
                if (gamepad1.y) {
                    Robot.shooterOn(Constants.powerConstant);
                }
                if (gamepad1.a) {
                    Robot.shooterOff();
                }
                if (gamepad1.left_bumper){ //code for speed change
                    Robot.SpeedControl(0.6);


                }
                else{
                    Robot.SpeedControl(1);
                }
                if (gamepad1.x){
                    Robot.ShootGoal();

                }
                if (gamepad1.b){
                    Robot.ShootOne();

                }
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