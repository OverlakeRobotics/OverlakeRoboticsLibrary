package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.components.AprilTagOdometry;
import org.firstinspires.ftc.teamcode.components.IMUOdometry;
import org.firstinspires.ftc.teamcode.components.SparkFunOTOSOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryCollection;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryModule;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Odometry OpMode", group = "TeleOp")
public class OdometryOpMode extends OpMode {

    public static int distance = 2160;
    public static double direction = 0;
    public static double velocity = 1000;
    private static final Pose2D STARTING_POSITION = new Pose2D(DistanceUnit.INCH, 60, -49, AngleUnit.DEGREES, 0);
    private static final Position cameraPosition = new Position(DistanceUnit.INCH,
            -9.5, 0, 2.6, 0);
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, -90, 0, 0);
    private final ElapsedTime runtime = new ElapsedTime();
    private VisionPortal visionPortal;
    private OdometryHolonomicDrivetrain driveTrain;
    private List<Pose2D> waypoints;
    private int currentWaypoint = 0;
    private double lastTime;

    @Override
    public void init() {
        List<OdometryModule> odometryModules = new ArrayList<>();

        SparkFunOTOS photoSensor = hardwareMap.get(SparkFunOTOS.class, "photosensor");
        configureOtos(photoSensor);

        // IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        IMU gyro = hardwareMap.get(IMU.class, "imu");
//        gyro.initialize(params);
//        Log.d("Heading", "Gyro Heading: " + gyro.getRobotYawPitchRollAngles().getYaw());

        odometryModules.add(new IMUOdometry(gyro));
        odometryModules.add(new SparkFunOTOSOdometry(photoSensor));

//        WebcamName camera = hardwareMap.get(WebcamName.class, "webcam");

//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
//                .setCameraPose(cameraPosition, cameraOrientation)
//                .build();

//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(camera);
//        builder.addProcessor(aprilTag);

//        visionPortal = builder.build();

//        odometryModules.add(new AprilTagOdometry(aprilTag, STARTING_POSITION));

        OdometryModule odometryCollection = new OdometryCollection(odometryModules);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                odometryCollection
        );
        odometryCollection.updatePosition();

        waypoints = new ArrayList<>();
        waypoints.add(new Pose2D(DistanceUnit.INCH, 0, 70, AngleUnit.DEGREES, 0));
        //waypoints.add(new Pose2D(DistanceUnit.INCH, -90, 90, AngleUnit.DEGREES, 0));
//        waypoints.add(new Pose2D(DistanceUnit.INCH, -85, 0, AngleUnit.DEGREES, 0));
//        waypoints.add(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        double currentTime = runtime.seconds();
        Log.d("Time", "Update Position Time: " + (runtime.seconds() - currentTime));
        currentTime = runtime.seconds();
        driveTrain.drive();
        Log.d("Time", "Drive Time: " + (runtime.seconds() - currentTime));
        if (driveTrain.isStopped() && currentWaypoint + 1 < waypoints.size()) {
            currentWaypoint++;
            driveTrain.setPositionDrive(waypoints.get(currentWaypoint), velocity);
        }
//        driveTrain.setPositionDrive(waypoints.get(currentWaypoint), velocity);
        Pose2D robotPosition = driveTrain.getPosition();
        currentTime = runtime.seconds();
        double dt = currentTime - lastTime;
        Log.d("Time", "Total delta time: " + dt);
        lastTime = currentTime;
//        telemetry.addData("Delta Time", dt);
//        telemetry.addData("X", robotPosition.getX(DistanceUnit.INCH));
//        telemetry.addData("Y", robotPosition.getY(DistanceUnit.INCH));
//        telemetry.addData("Heading", robotPosition.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("", "");
//        telemetry.addData("Distance Left", driveTrain.getPositionDriveDistanceLeft());
//        telemetry.addData("Direction", driveTrain.getPositionDriveDirection());
//        telemetry.addData("", "");
//        telemetry.addData("Forward Counts Left", driveTrain.getForwardCountsLeft());
//        telemetry.addData("Strafe Counts Left", driveTrain.getStrafeCountsLeft());
//        telemetry.addData("Turn Counts Left", driveTrain.getTurnCountsLeft());
//        telemetry.update();
    }

    private void configureOtos(SparkFunOTOS photoSensor) {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        photoSensor.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AngularUnit.RADIANS);
        photoSensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 45);
        photoSensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        photoSensor.setLinearScalar(1.077);
        photoSensor.setAngularScalar(0.993);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        photoSensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        photoSensor.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        photoSensor.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        photoSensor.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    @Override
    public void start() {
        driveTrain.updatePosition();
        driveTrain.setPositionDriveCorrection(distance, direction, velocity, 0);
//        driveTrain.setPositionDrive(waypoints.get(currentWaypoint), velocity);
//        driveTrain.setPositionDrive(distance, direction, velocity, driveTrain.getPosition().getHeading(AngleUnit.DEGREES));
    }
}
