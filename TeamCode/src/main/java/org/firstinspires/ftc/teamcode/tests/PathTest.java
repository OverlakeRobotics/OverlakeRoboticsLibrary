package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.components.IMUOdometry;
import org.firstinspires.ftc.teamcode.components.SparkFunOTOSOdometry;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "Path Test", group = "TeleOp")
public class PathTest extends OpMode {
    public static class Offsets {
        public double yOffset = -168.0; // mm
        public double xOffset = -84.0; // mm
    }

    public static Offsets OFFSETS = new Offsets();

    public static int velocity = 2000;

    private OdometryHolonomicDrivetrain driveTrain;

    public Pose2D startPos = new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 0);

    public final Pose2D[] positions = new Pose2D[]{
            new Pose2D(DistanceUnit.INCH, -63.0, 15.0, AngleUnit.DEGREES, 0.0),
            new Pose2D(DistanceUnit.INCH, -56.735, 15.411, AngleUnit.DEGREES, 7.5),
            new Pose2D(DistanceUnit.INCH, -50.577, 16.636, AngleUnit.DEGREES, 15.0),
            new Pose2D(DistanceUnit.INCH, -44.631, 18.654, AngleUnit.DEGREES, 22.5),
            new Pose2D(DistanceUnit.INCH, -39.0, 21.431, AngleUnit.DEGREES, 30.0),
            new Pose2D(DistanceUnit.INCH, -33.779, 24.919, AngleUnit.DEGREES, 37.5),
            new Pose2D(DistanceUnit.INCH, -29.059, 29.059, AngleUnit.DEGREES, 45.0),
            new Pose2D(DistanceUnit.INCH, -24.919, 33.779, AngleUnit.DEGREES, 52.5),
            new Pose2D(DistanceUnit.INCH, -21.431, 39.0, AngleUnit.DEGREES, 60.0),
            new Pose2D(DistanceUnit.INCH, -18.654, 44.631, AngleUnit.DEGREES, 67.5),
            new Pose2D(DistanceUnit.INCH, -16.636, 50.577, AngleUnit.DEGREES, 75.0),
            new Pose2D(DistanceUnit.INCH, -15.411, 56.735, AngleUnit.DEGREES, 82.5),
            new Pose2D(DistanceUnit.INCH, -15.0, 63.0, AngleUnit.DEGREES, 90.0),
            new Pose2D(DistanceUnit.INCH, -15.411, 69.265, AngleUnit.DEGREES, 97.5),
            new Pose2D(DistanceUnit.INCH, -16.636, 75.423, AngleUnit.DEGREES, 105.0),
            new Pose2D(DistanceUnit.INCH, -18.654, 81.369, AngleUnit.DEGREES, 112.5),
            new Pose2D(DistanceUnit.INCH, -21.431, 87.0, AngleUnit.DEGREES, 120.0),
            new Pose2D(DistanceUnit.INCH, -24.919, 92.221, AngleUnit.DEGREES, 127.5),
            new Pose2D(DistanceUnit.INCH, -29.059, 96.941, AngleUnit.DEGREES, 135.0),
            new Pose2D(DistanceUnit.INCH, -33.779, 101.081, AngleUnit.DEGREES, 142.5),
            new Pose2D(DistanceUnit.INCH, -39.0, 104.569, AngleUnit.DEGREES, 150.0),
            new Pose2D(DistanceUnit.INCH, -44.631, 107.346, AngleUnit.DEGREES, 157.5),
            new Pose2D(DistanceUnit.INCH, -50.577, 109.364, AngleUnit.DEGREES, 165.0),
            new Pose2D(DistanceUnit.INCH, -56.735, 110.589, AngleUnit.DEGREES, 172.5),
            new Pose2D(DistanceUnit.INCH, -63.0, 111.0, AngleUnit.DEGREES, -180.0),
    };

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(OFFSETS.xOffset, OFFSETS.yOffset);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
        driveTrain.setPosition(startPos);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.drive();
    }

    @Override
    public void start() {
        driveTrain.setPositionDrive(positions, velocity);
    }
}
