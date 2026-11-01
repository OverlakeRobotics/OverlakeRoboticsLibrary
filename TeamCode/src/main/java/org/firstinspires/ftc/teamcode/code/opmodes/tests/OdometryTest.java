package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// This OpMode shows a simple example of driving the bot along a set path of points using the
// driveTrain.setPositionDrive() method while passing in an array of Pose2Ds.
@Config
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends OpMode {
    // Change to your actual offsets. See GoBildaPinpointDriver.setOffsets() for details on measuring offsets.
    public double yOffset = -156.0;
    public double xOffset = -84.0;

    private OdometryHolonomicDrivetrain driveTrain;
    private GoBildaPinpointDriver pinpointDriver;

    private final Pose2D[] drivePoints1 = {
            new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0),
    };

    private final Pose2D[] drivePoints2 = {
            new Pose2D(DistanceUnit.INCH, 72, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 180)
    };

    private final Pose2D[] drivePoints3 = {
            new Pose2D(DistanceUnit.INCH, 72, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, -90),
            new Pose2D(DistanceUnit.INCH, 0, 72, AngleUnit.DEGREES, 180),
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90),
    };

    private final Pose2D[] drivePoints4 = {
            new Pose2D(DistanceUnit.INCH, 72, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)
    };

    private final Pose2D[] drivePoints5 = {
            new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 0),
    };

    public static int velocity = 2000;
    public static double tolerance = 1;

    private final Pose2D[][] paths = {drivePoints1, drivePoints2, drivePoints3, drivePoints4, drivePoints5};
    private int pathIndex = 0;
    private Pose2D[] path = paths[pathIndex];
    private boolean running = false;

    private final ElapsedTime pauseTimer = new ElapsedTime();
    private int pointIndex = 0;
    private boolean pausing = false;
    public static double pauseSeconds = 2.0;

    @Override
    public void init() {
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        driveTrain.setTolerance(tolerance);
        driveTrain.setVelocity(velocity);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.drive();

        if (!running) {
            if (gamepad1.xWasPressed()) {
                pathIndex = Math.max(0, pathIndex - 1);
            } else if (gamepad1.bWasPressed()) {
                pathIndex = Math.min(paths.length - 1, pathIndex + 1);
            }
            path = paths[pathIndex];

            if (gamepad1.dpadLeftWasPressed()) {
                pauseSeconds = Math.max(0, pauseSeconds + 0.5);
            } else if (gamepad1.dpadRightWasPressed()) {
                pauseSeconds = Math.max(0, pauseSeconds - 0.5);
            } else if (gamepad1.dpadUpWasPressed()) {
                velocity += 100;
                driveTrain.setVelocity(velocity);
            } else if (gamepad1.dpadDownWasPressed()) {
                velocity = Math.max(0, velocity - 100);
                driveTrain.setVelocity(velocity);
            }

            if (gamepad1.rightBumperWasPressed()) {
                driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                pointIndex = 0;
                pausing = false;
                driveTrain.setVelocity(velocity);
                driveTrain.setPositionDrive(path[pointIndex]);
                running = true;
            }
        }

        if (running) {
            if (pausing) {
                if (pauseTimer.seconds() >= pauseSeconds) {
                    pointIndex++;
                    driveTrain.setVelocity(velocity);
                    driveTrain.setPositionDrive(path[pointIndex]);
                    pausing = false;
                }
            } else if (!driveTrain.isDriving()) {
                driveTrain.stop();

                if (pointIndex < path.length - 1) {
                    pauseTimer.reset();
                    pausing = true;
                } else {
                    running = false;
                }
            }
        }

        telemetry.addData("path", "%d / %d", pathIndex + 1, paths.length);
        telemetry.addData("velocity:", "%d ticks/sec", velocity);
        telemetry.addData("pause seconds", "%.2f", pauseSeconds);
        telemetry.addData("point index", "%d", pointIndex);
        Pose2D position = driveTrain.getPosition();
        telemetry.addData("position", "X: %.2f in, Y: %.2f in, Heading: %.2f deg",
                position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH),
                position.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        // wait for user to start
    }
}
