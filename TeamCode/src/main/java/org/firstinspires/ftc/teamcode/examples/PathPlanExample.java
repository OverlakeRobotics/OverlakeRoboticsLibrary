package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

import java.util.Arrays;

@Config
@Autonomous(name = "Path Planning Example", group = "Autonomous")
public class PathPlanExample extends OpMode {
    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm

    public static int velocity;
    public static double tolerance;

    private OdometryHolonomicDrivetrain driveTrain;
    public Pose2D[] positions;
    public PathServer.Tag[] tags;
    private int lastTagIndex = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = -1;

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        PathServer.startServer();
    }

    @Override
    public void init_loop() {
        driveTrain.updatePosition();
        PathServer.setRobotPose(driveTrain.getPosition());
    }

    @Override
    public void start() {
        velocity = (int) (PathServer.getVelocity() * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH);
        tolerance = PathServer.getTolerance();
        driveTrain.setTolerance(tolerance);
        positions = PathServer.getPath();
        driveTrain.setPosition(PathServer.getStartPose());
        tags = PathServer.getTags();
        Arrays.sort(tags);
        driveTrain.setPositionDrive(positions, velocity);
    }


    @Override
    public void loop() {
        driveTrain.updatePosition();
        PathServer.setRobotPose(driveTrain.getPosition());
        if (pauseTimeLeft <= 0) {
            driveTrain.drive();
            int nextPointIndex = driveTrain.getNextPointIndex();

            while (lastTagIndex < tags.length && tags[lastTagIndex].index <= nextPointIndex) {
                PathServer.Tag currTag = tags[lastTagIndex];
                switch (currTag.name) {
                    case "velocity":
                        driveTrain.setVelocity((int) (currTag.value * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH));
                        break;
                    case "pause":
                        pauseTimeLeft += currTag.value;
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1], velocity);
                        break;
                }
                lastTagIndex++;
            }
        } else {
            pauseTimeLeft -= runtime.seconds() - lastTime;
            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                driveTrain.setPositionDrive(positions, velocity, pausedIndex);
            }
        }
        lastTime = runtime.seconds();
    }

    @Override
    public void stop() {
        PathServer.stopServer();
    }
}
