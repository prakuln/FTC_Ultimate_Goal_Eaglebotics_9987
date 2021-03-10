package org.firstinspires.ftc.teamcode.OpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Trajectories.Autocode;
import org.firstinspires.ftc.teamcode.Vision.Camera;
//THIS IS A BETA VERSION FOR THE AUTONOMOUS CODE! ALL TESTINGS OCCUR HERE. NOT FOR USE IN COMPETITIONS!

@Autonomous(name = "Auto_Beta", group = "OpModes")

public class Auto_Beta extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
   public void runOpMode(){
        Robot.initAuto(hardwareMap, telemetry);
        waitForStart();

        if (opModeIsActive()) {
            switch (Camera.getHeight()) {
                case ZERO:
                    Autocode.a(telemetry);
                    break;
                case ONE:
                    Autocode.b(telemetry);
                    break;
                case FOUR:
                    Autocode.c(telemetry);
                    break;
            }

        }

    }
}


