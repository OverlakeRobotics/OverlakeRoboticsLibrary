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
@TeleOp(name = "Odometry OpMode", group = "TeleOp")
public class OdometryOpMode extends OpMode {

    public static int velocity = 2000;
    private OdometryHolonomicDrivetrain driveTrain;

    private final Pose2D[] drivePoints = {
            new Pose2D(DistanceUnit.INCH, 106, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 106, -85, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 5, -85, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 5, 0, AngleUnit.DEGREES, 0),
    };
    private int currentPoint = 0;

    @Override
    public void init() {
        IMU gyro = hardwareMap.get(IMU.class, "imu");
        // Change this to match orientation of IMU on the robot
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        gyro.initialize(params);
        gyro.resetYaw();

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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

        Pose2D pos = driveTrain.getPosition();
        Log.d("Position", "X: " + pos.getX(DistanceUnit.INCH) + ", Y: " + pos.getY(DistanceUnit.INCH) + ", Heading: " + pos.getHeading(AngleUnit.DEGREES));

        if (driveTrain.isStopped()) {
            currentPoint++;
            if (currentPoint < drivePoints.length) {
                driveTrain.setPositionDrive(drivePoints[currentPoint], velocity);
            }
        }
    }

    @Override
    public void start() {
        driveTrain.updatePosition();
//        driveTrain.setPositionDriveCorrection(2500, 0, velocity, 30);
        driveTrain.setPositionDrive(drivePoints[currentPoint], velocity);
    }
}
