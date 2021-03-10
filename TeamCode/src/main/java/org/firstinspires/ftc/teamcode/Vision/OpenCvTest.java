package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OpenCV Test", group = "OpModes")
public class OpenCvTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Camera.init(hardwareMap, telemetry);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("[Ring Stack] >>", Camera.getHeight());
                telemetry.update();

            }

        }

    }
}
