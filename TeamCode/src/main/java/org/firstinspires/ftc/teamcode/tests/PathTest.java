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

    private final Pose2D[] positions = {
            new Pose2D(DistanceUnit.INCH,   0.00,   0.00, AngleUnit.DEGREES,   0.0),
            new Pose2D(DistanceUnit.INCH,   1.64,  12.42, AngleUnit.DEGREES,  15.0),
            new Pose2D(DistanceUnit.INCH,   6.43,  24.00, AngleUnit.DEGREES,  30.0),
            new Pose2D(DistanceUnit.INCH,  14.06,  33.94, AngleUnit.DEGREES,  45.0),
            new Pose2D(DistanceUnit.INCH,  24.00,  41.57, AngleUnit.DEGREES,  60.0),
            new Pose2D(DistanceUnit.INCH,  35.58,  46.36, AngleUnit.DEGREES,  75.0),
            new Pose2D(DistanceUnit.INCH,  48.00,  48.00, AngleUnit.DEGREES,  90.0),
            new Pose2D(DistanceUnit.INCH,  60.42,  46.36, AngleUnit.DEGREES, 105.0),
            new Pose2D(DistanceUnit.INCH,  72.00,  41.57, AngleUnit.DEGREES, 120.0),
            new Pose2D(DistanceUnit.INCH,  81.94,  33.94, AngleUnit.DEGREES, 135.0),
            new Pose2D(DistanceUnit.INCH,  89.57,  24.00, AngleUnit.DEGREES, 150.0),
            new Pose2D(DistanceUnit.INCH,  94.36,  12.42, AngleUnit.DEGREES, 165.0),
            new Pose2D(DistanceUnit.INCH,  96.00,   0.00, AngleUnit.DEGREES, 180.0)
    };

    @Override
    public void init() {
        IMU gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        gyro.initialize(params);
        gyro.resetYaw();

        // Swapped because I got it wrong at first
        for (int i = 0; i < positions.length; i++) {
            positions[i] = new Pose2D(DistanceUnit.INCH, positions[i].getY(DistanceUnit.INCH), positions[i].getX(DistanceUnit.INCH), AngleUnit.DEGREES, positions[i].getHeading(AngleUnit.DEGREES));
        }

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(OFFSETS.xOffset, OFFSETS.yOffset);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
//                new IMUOdometry(gyro)
                new GoBildaPinpointOdometry(pinpointDriver)
        );
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.drive();
    }

    @Override
    public void start() {
        driveTrain.updatePosition();
//        driveTrain.setPositionDriveCorrection(2500, 0, velocity, 30);
        driveTrain.setPositionDrive(positions, velocity);
//        driveTrain.setPositionDrive(wantedPosition, velocity);
    }
}
