package org.firstinspires.ftc.teamcode.system;


import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;


// The list of what your robot does at each Path Planner tag. PathPlanAuto gives you one of these
// through tag() and stopTag(), so you normally never touch this class directly.
//
// It turns each entry into the Ivy command and the stop that PathPlanRunner and PedroPathBuilder
// need. "velocity" and "pause" are built in and do not go in the list.
public class TagList implements PathPlanRunner.TagHandler, PedroPathBuilder.SegmentPolicy {

    // One tag that takes time. The tag's value is passed to start(). Without until() it runs for
    // <value> seconds. A value of 0 or less skips it, so you can turn a tag off in the planner.
    public static final class Task {
        private final boolean stops;
        private final boolean once;
        private DoubleConsumer start = value -> {};
        private Runnable loop = () -> {};
        private BooleanSupplier until = null;
        private Runnable end = () -> {};

        private Task(boolean stops, boolean once) {
            this.stops = stops;
            this.once = once;
        }

        // Runs once when the tag is reached.
        public Task start(DoubleConsumer action) {
            start = action;
            return this;
        }

        public Task start(Runnable action) {
            start = value -> action.run();
            return this;
        }

        // Runs every loop while the tag is active.
        public Task loop(Runnable action) {
            loop = action;
            return this;
        }

        // Finishes the tag as soon as this is true, instead of after <value> seconds.
        public Task until(BooleanSupplier done) {
            until = done;
            return this;
        }

        // Runs once when the tag finishes.
        public Task end(Runnable action) {
            end = action;
            return this;
        }

        private boolean isActive(double value) {
            return once || value > 0;
        }

        private Command command(double value) {
            if (!isActive(value)) return null;
            if (once) return Commands.instant(() -> start.accept(value));

            ElapsedTime timer = new ElapsedTime();
            return Command.build()
                    .setStart(() -> {
                        timer.reset();
                        start.accept(value);
                    })
                    .setExecute(() -> loop.run())
                    .setDone(() -> until != null ? until.getAsBoolean() : timer.seconds() >= value)
                    .setEnd(interrupted -> end.run());
        }
    }

    private final Map<String, Task> tasks = new LinkedHashMap<>();

    // Runs action once when the tag is reached. The robot keeps driving.
    public void add(String name, Runnable action) {
        add(name, value -> action.run());
    }

    // Same, and action gets the tag's value from the planner.
    public void add(String name, DoubleConsumer action) {
        tasks.put(name, new Task(false, true).start(action));
    }

    // A tag that takes time while the robot keeps driving. The next tag or stop waits for it.
    public Task addTask(String name) {
        Task task = new Task(false, false);
        tasks.put(name, task);
        return task;
    }

    // A tag that stops the robot at its point until the task finishes, then drives on.
    public Task addStop(String name) {
        Task task = new Task(true, false);
        tasks.put(name, task);
        return task;
    }

    @Override
    public Command commandFor(String name, double value) {
        Task task = tasks.get(name);
        return task != null ? task.command(value) : null;
    }

    @Override
    public boolean stopsBefore(int segmentIndex, PathRoute.Segment segment, List<PathServer.Tag> tags) {
        if (PedroPathBuilder.PAUSE_ONLY.stopsBefore(segmentIndex, segment, tags)) return true;
        for (PathServer.Tag t : tags) {
            Task task = tasks.get(t.name);
            if (task != null && task.stops && task.isActive(t.value)) return true;
        }
        return false;
    }

    // Tag names on the route that nothing handles, so a typo does not silently do nothing.
    public List<String> ignoredTags(PathRoute route) {
        List<String> names = new ArrayList<>();
        for (PathServer.Tag t : route.tags) {
            boolean builtIn = PathPlanRunner.TAG_VELOCITY.equals(t.name) || PedroPathBuilder.TAG_PAUSE.equals(t.name);
            if (!builtIn && !tasks.containsKey(t.name) && !names.contains(t.name)) names.add(t.name);
        }
        return names;
    }
}
