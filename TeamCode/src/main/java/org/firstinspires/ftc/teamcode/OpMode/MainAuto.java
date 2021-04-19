package org.firstinspires.ftc.teamcode.OpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Trajectories.Autocode;
import org.firstinspires.ftc.teamcode.Vision.Camera;

@Autonomous(name = "MainAuto", group = "OpModes")

public class MainAuto extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Robot.initAuto(hardwareMap, telemetry);
        waitForStart();
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


