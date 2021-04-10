package org.firstinspires.ftc.teamcode.OpMode;

//THIS IS A BETA VERSION FOR THE TELEOP CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Joystick;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Disabled
@TeleOp(name = "TestHardware", group = "OpModes")
public class TestHardware extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot.initTeleOp(hardwareMap, telemetry);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Joystick.testControl(gamepad1);
            }
        }
    }
}
