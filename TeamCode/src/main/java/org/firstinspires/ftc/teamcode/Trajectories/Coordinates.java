package org.firstinspires.ftc.teamcode.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Coordinates {
    public static Pose2d start = new Pose2d(-60,-60, Math.toRadians(0));
    public static Pose2d auto_point = new Pose2d(-12,-60, Math.toRadians(0));
    public static Pose2d shoot = new Pose2d(1, -39, Math.toRadians(3));
    public static Pose2d a = new Pose2d(12,-53, Math.toRadians(270));
    public static Pose2d b = new Pose2d(20, -39, Math.toRadians(0));
    public static Pose2d c = new Pose2d(45, -50, Math.toRadians(315));
    public static Pose2d wobble = new Pose2d(-40,-29, Math.toRadians(180));
    //public static Pose2d zero = new Pose2d(60, shoot.getY(), Math.toRadians(0));
    public static Pose2d end = new Pose2d();
}
