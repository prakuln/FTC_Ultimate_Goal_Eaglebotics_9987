package org.firstinspires.ftc.teamcode.Robot;

public class Camera {
    public static void out(){
        Robot.cam.setPosition(0.935);
    }
    public static void in(){
        Robot.cam.setPosition(0.2);
    }
    public static void setZoom(){
        if (Robot.tfod != null) {
            Robot.tfod.activate();
            Robot.tfod.setZoom(2, 1.3);
        }
    }
}
