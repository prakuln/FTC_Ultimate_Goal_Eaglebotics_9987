package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Coordinates {
    static Pose2d start = new Pose2d(-60,-60, Math.toRadians(0));
    static Pose2d auto_point = new Pose2d(-12,-60, Math.toRadians(0));
    static Pose2d shoot = new Pose2d(1, -40, Math.toRadians(358));
    static Pose2d a = new Pose2d(12,-50, Math.toRadians(270));
    static Pose2d b = new Pose2d(20, -39, Math.toRadians(0));
    static Pose2d c = new Pose2d(45, -50, Math.toRadians(315));
    static Pose2d wobble = new Pose2d(-41,-28,Math.toRadians(180));
    static Pose2d zero = new Pose2d(60, shoot.getY(), Math.toRadians(0));
    static Pose2d end = new Pose2d();
}
