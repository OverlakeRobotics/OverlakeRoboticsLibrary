package org.firstinspires.ftc.teamcode.code.opmodes.autons;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Pusher;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.code.helpers.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

import java.util.Arrays;

@Config
@Autonomous(name = "Basic Path Planner Auto", group = "Autonomous", preselectTeleOp = "Basic TeleOp") //settings > opmode auto q
public class BasicPathPlannerAuto extends OpMode {
    private OdometryHolonomicDrivetrain driveTrain;
    private Intake intake;
    private Shooter shooter;
    private Pusher pusher;

    public Pose2D[] positions;
    public PathServer.Tag[] tags;
    private int lastTagIndex = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = -1;
    private int shotsPushed = 0;
    private boolean shooting = false;
    private boolean routeFinished = false;
    private boolean autoStarted = false;
    private static double SHOOTER_RPM = 3000;

    @Override
    public void init() {
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"))
        );

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        shooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        pusher = new Pusher(hardwareMap.get(Servo.class, "pusher"));

        PathServer.startServer();
    }

    @Override
    public void init_loop() {
        driveTrain.updatePosition();
        PathServer.setRobotPose(driveTrain.getPosition());
    }

    @Override
    public void start() {
        autoStarted = true;
        runtime.reset();
        lastTime = 0;
        pauseTimeLeft = 0;
        driveTrain.setVelocity((int) (PathServer.getVelocity() * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH));
        positions = PathServer.getPath();
        driveTrain.setPosition(PathServer.getStartPose());
        driveTrain.setTolerance(PathServer.getTolerance());

        tags = PathServer.getTags();
        Arrays.sort(tags);

        driveTrain.setPositionDrive(positions);
    }


    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D pos = driveTrain.getPosition();
        PathServer.setRobotPose(pos);

        double curTime = runtime.seconds();
        double dt = curTime - lastTime;
        lastTime = curTime;

        if (routeFinished) return;

        if (shooting) {
            driveTrain.setPositionDrive(pausedIndex == 0
                    ? PathServer.getStartPose() : positions[pausedIndex - 1]);
            boolean pusherWasBusy = pusher.getState() != 0;
            pusher.update(false);
            if (pusherWasBusy && pusher.getState() == 0) {
                shootTimer.reset();
            }

            if (shotsPushed < 5 && pusher.getState() == 0 && shootTimer.milliseconds() >= 1000) {
                pusher.update(true);
                shotsPushed++;
            }

            if (shotsPushed == 5 && pusher.getState() == 0) {
                intake.setIntakePower(0);
                shooter.setShooterVelocity(0);
                shooting = false;
                if (pausedIndex < positions.length) {
                    driveTrain.setPositionDrive(positions, pausedIndex);
                } else {
                    routeFinished = true;
                    driveTrain.stop();
                }
            }
            return;
        }

        if (pauseTimeLeft > 0) {
            pauseTimeLeft -= dt;
            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                if (pausedIndex < positions.length) {
                    driveTrain.setPositionDrive(positions, pausedIndex);
                } else {
                    routeFinished = true;
                    driveTrain.stop();
                }
            } else {
                driveTrain.setPositionDrive(pausedIndex == 0
                        ? PathServer.getStartPose() : positions[pausedIndex - 1]);
            }
            return;
        }

        driveTrain.drive();
        int nextPointIndex = driveTrain.getNextPointIndex();
        int reachedPointIndex = nextPointIndex;
        if (nextPointIndex == positions.length - 1 && !driveTrain.isDriving()) {
            reachedPointIndex = positions.length;
        }

        while (lastTagIndex < tags.length && tags[lastTagIndex].index <= reachedPointIndex) {
            PathServer.Tag currTag = tags[lastTagIndex++];
            switch (currTag.name) {
                case "velocity":
                    driveTrain.setVelocity((int) (currTag.value * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH));
                    break;
                case "pause":
                    if (currTag.value <= 0) break;
                    pauseTimeLeft = currTag.value;
                    pausedIndex = Math.max(0, Math.min(currTag.index, positions.length));
                    driveTrain.setPositionDrive(pausedIndex == 0
                            ? PathServer.getStartPose() : positions[pausedIndex - 1]);
                    break;
                case "shoot":
                    if (currTag.value > 0) {
                        pausedIndex = Math.max(0, Math.min(currTag.index, positions.length));
                        shotsPushed = 0;
                        shooting = true;
                        intake.setIntakePower(0);
                        shooter.setShooterVelocity(SHOOTER_RPM * 28.0 / 60.0);
                        shootTimer.reset();
                        driveTrain.stop();
                    }
                    break;
                case "reset":
                    pusher.home();
                    break;
                default:
                    break;
            }
            if (shooting || pauseTimeLeft > 0) break;
        }
    }

    @Override
    public void stop() {
        if (autoStarted) {
            driveTrain.updatePosition();
            AutoPoseHandoff.save(driveTrain.getPosition());
        }
        intake.setIntakePower(0);
        shooter.setShooterVelocity(0);
        pusher.home();
        PathServer.stopServer();
    }
}
