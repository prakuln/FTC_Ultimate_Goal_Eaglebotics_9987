package org.firstinspires.ftc.teamcode;

//THIS IS A BETA VERSION FOR THE TELEOP CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestHardare", group = "")
public class TestHardware extends LinearOpMode {
    public static DcMotor leftFront;
    public static DcMotor leftRear;
    public static DcMotor rightFront;
    public static DcMotor rightRear;
    private DcMotor MRightShooter;
    private DcMotor MLeftShooter;
    private DcMotor MIntake;
    private DcMotor MArm;
    private Servo LClaw;
    private Servo RClaw;
    private Servo Cam;
    private Servo Hopper;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        Robot.leftRear = hardwareMap.dcMotor.get("leftRear");
        Robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        Robot.rightRear = hardwareMap.dcMotor.get("rightRear");
        MLeftShooter = hardwareMap.dcMotor.get("MLeftShooter");
        MRightShooter = hardwareMap.dcMotor.get("MRightShooter");
        MIntake = hardwareMap.dcMotor.get("MIntake");
        MArm = hardwareMap.dcMotor.get("MArm");
        LClaw = hardwareMap.servo.get("LClaw");
        RClaw = hardwareMap.servo.get("RClaw");
        Cam = hardwareMap.servo.get("Cam");
        Hopper = hardwareMap.servo.get("Hopper");
        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {
                leftFront.setPower(gamepad1.left_stick_y);
                leftRear.setPower(gamepad1.left_stick_x);
                rightFront.setPower(gamepad1.right_stick_y);
                rightRear.setPower(gamepad1.right_stick_x);
            }
        }
    }
}
