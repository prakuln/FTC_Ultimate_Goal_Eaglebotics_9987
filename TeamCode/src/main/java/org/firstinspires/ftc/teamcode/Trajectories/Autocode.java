package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Camera;

public class Autocode {
    public static void a(Telemetry telemetry){
        telemetry.addData("Target Zone", "A");
        telemetry.update();
        Camera.in();
        Navigation.goToShoot();
        Navigation.goToZone(0);
        Navigation.getWobble(0);
        Navigation.bringWobble(0);
        Navigation.goToLine(0);
        Drivetrain.setEndPose();
    }
    public static void b(Telemetry telemetry){
        telemetry.addData("Target Zone", "B");
        telemetry.update();
        Camera.in();
        Navigation.goToShoot();
        Navigation.shootStack(1);
        Navigation.goToZone(1);
        Navigation.getWobble(1);
        Navigation.bringWobble(1);
        Navigation.goToLine(1);
        Drivetrain.setEndPose();
    }
    public static void c(Telemetry telemetry){
        telemetry.addData("Target Zone", "C");
        telemetry.update();
        Camera.in();
        Navigation.goToShoot();
        Navigation.shootStack(4);
        Navigation.goToZone(4);
        Navigation.getWobble(4);
        Navigation.bringWobble(4);
        Navigation.goToLine(4);
        Drivetrain.setEndPose();
    }
}
