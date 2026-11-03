package org.firstinspires.ftc.teamcode.system;


import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.List;
import java.util.function.DoubleConsumer;


// Extend this to run Path Planner routes on your robot with Pedro Pathing. You only write two
// things: setup() to get your hardware, and tags() with one line per tag:
//
//     @Override
//     protected void tags() {
//         tag("intake", power -> intake.setPower(power));      // once, gets the tag's value
//         tag("myTag", () -> mechanism.go());                  // once
//         stopTag("myStopTag")                                 // stops the robot until finished
//                 .start(() -> mechanism.begin())
//                 .loop(() -> mechanism.update())
//                 .until(() -> mechanism.isDone())             // leave out to run <value> seconds
//                 .end(() -> mechanism.finish());
//     }
//
// "velocity" and "pause" are built in. Any other tag on the route that is not in your list is shown
// in telemetry as ignored. Add new tag names to config.js in the planner so you can place them.
//
// By default the OpMode runs whatever route you upload from the planner. To run a saved route with
// no laptop at the competition, export it to TeamCode/src/main/assets and override routeFile().
public abstract class PathPlanAuto extends OpMode {
    // Where the robot was when Auto ended, so a TeleOp can start from there. Lost on app restart.
    public static Pose2D lastPose = null;

    protected Follower follower;
    protected String alliance;   // "red", "blue" or "unknown", set when the OpMode starts

    private final TagList tagList = new TagList();
    private FollowerOdometryModule odometry;
    private PathRoute route;
    private Command routeCommand;

    // Get your hardware from hardwareMap here.
    protected abstract void setup();

    // List what your robot does at each tag with tag() and stopTag().
    protected abstract void tags();

    // Called every loop while the route runs.
    protected void update() {}

    // Called when the OpMode stops. Turn your mechanisms off here.
    protected void finish() {}

    // Saved route to run instead of an uploaded one, e.g. "pathJsons/closeBlue.json".
    protected String routeFile() {
        return null;
    }

    // Runs action once when the tag is reached. The robot keeps driving.
    protected final void tag(String name, Runnable action) {
        tagList.add(name, action);
    }

    // Same, and action gets the tag's value from the planner.
    protected final void tag(String name, DoubleConsumer action) {
        tagList.add(name, action);
    }

    // A tag that takes time while the robot keeps driving. Chain start/loop/until/end on it.
    protected final TagList.Task tag(String name) {
        return tagList.addTask(name);
    }

    // A tag that stops the robot at its point until finished. Chain start/loop/until/end on it.
    protected final TagList.Task stopTag(String name) {
        return tagList.addStop(name);
    }

    // The robot's current position on the field.
    protected final Pose2D getPose() {
        return odometry.getPosition();
    }

    @Override
    public final void init() {
        Scheduler.reset();   // the scheduler is static, so clear anything left from a previous OpMode
        follower = Constants.create(hardwareMap);
        odometry = new FollowerOdometryModule(follower);
        if (Constants.PEDRO_TUNED) PathPlanRunner.publishRobotLimits(follower);   // real speeds for the planner preview

        setup();
        tags();

        if (routeFile() != null) route = readRouteFile();
        else PathServer.startServer();
    }

    @Override
    public final void init_loop() {
        // Show the planner where the robot thinks it is.
        follower.update();
        PathServer.setRobotPose(getPose());

        PathRoute shown = currentRoute();
        if (!Constants.PEDRO_TUNED) telemetry.addLine("Pedro is not tuned yet, see pedro/Constants.java");
        telemetry.addData("Route", shown.segments.size() + " segments, " + shown.tags.size() + " tags");
        addIgnoredTags(shown);
        telemetry.update();
    }

    @Override
    public final void start() {
        route = currentRoute();
        alliance = route.alliance;
        follower.setPose(PedroPathBuilder.toPose(route.start));

        PedroPathBuilder.Options options = new PedroPathBuilder.Options();
        options.policy = tagList;
        routeCommand = PathPlanRunner.build(follower, PedroPathBuilder.build(route, options), tagList);
        Scheduler.schedule(routeCommand);
    }

    @Override
    public final void loop() {
        follower.update();      // localizes and drives the motors
        Scheduler.execute();    // advances the route and any tags
        update();

        Pose2D pose = getPose();
        PathServer.setRobotPose(pose);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f", pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Route done", !routeCommand.isScheduled());
        addIgnoredTags(route);
        telemetry.update();
    }

    @Override
    public final void stop() {
        if (routeCommand != null) lastPose = getPose();
        Scheduler.reset();
        follower.stop();
        finish();
        PathServer.stopServer();
    }

    private PathRoute currentRoute() {
        return routeFile() != null ? route : PathServer.getRoute();
    }

    private void addIgnoredTags(PathRoute shown) {
        List<String> ignored = tagList.ignoredTags(shown);
        if (!ignored.isEmpty()) telemetry.addData("Ignored tags (not in your list)", ignored);
    }

    private PathRoute readRouteFile() {
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(hardwareMap.appContext.getAssets().open(routeFile())))) {
            StringBuilder json = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) json.append(line);
            return PathRoute.fromJson(new JSONObject(json.toString()));
        } catch (IOException | JSONException e) {
            throw new RuntimeException("Could not read route " + routeFile(), e);
        }
    }
}
