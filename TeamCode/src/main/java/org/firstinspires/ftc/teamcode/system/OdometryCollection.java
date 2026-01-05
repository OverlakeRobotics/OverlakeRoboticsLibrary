package org.firstinspires.ftc.teamcode.system;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;


public class OdometryCollection implements OdometryModule {
    List<OdometryModule> odometryModules;
    Pose2D position;
    private int positionPriority;
    private int headingPriority;
    private boolean doPositionReset;
    private boolean doHeadingReset;
    private boolean isPositionAccurate;
    private boolean isHeadingAccurate;
    public OdometryCollection(List<OdometryModule> odometryModules) {
        this.odometryModules = odometryModules;
        positionPriority = Integer.MIN_VALUE;
        headingPriority = Integer.MIN_VALUE;
        for (OdometryModule module : odometryModules) {
            if (module.getPositionPriority() > positionPriority) {
                positionPriority = module.getPositionPriority();
            }

            if (module.getHeadingPriority() > headingPriority) {
                headingPriority = module.getHeadingPriority();
            }
        }

        doPositionReset = false;
        doHeadingReset = false;
    }

    // Behavior: Updates the position of all odometry modules taking into account their priorities.
    public void updatePosition() {
        for (OdometryModule module : odometryModules) {
            module.updatePosition();
        }

        Pose2D highestPriorityPosition = null;
        int highestPositionPriority = Integer.MIN_VALUE;
        OdometryModule highestPositionPriorityModule = null;
        Pose2D highestPriorityHeading = null;
        int highestHeadingPriority = Integer.MIN_VALUE;
        OdometryModule highestHeadingPriorityModule = null;

        for (OdometryModule module : odometryModules) {
            if (module.isPositionAccurate() && module.getPositionPriority() > highestPositionPriority) {
                highestPriorityPosition = module.getPosition();
                highestPositionPriority = module.getPositionPriority();
                highestPositionPriorityModule = module;
            }

            if (module.isHeadingAccurate() && module.getHeadingPriority() > highestHeadingPriority) {
                highestPriorityHeading = module.getPosition();
                highestHeadingPriority = module.getHeadingPriority();
                highestHeadingPriorityModule = module;
            }
        }

        if (highestPriorityPosition == null) {
            isPositionAccurate = false;
            for (OdometryModule module : odometryModules) {
                if (module.getPositionPriority() == highestPositionPriority) {
                    highestPriorityPosition = module.getPosition();
                }
            }
        } else {
            isPositionAccurate = true;
        }

        if (highestPriorityHeading == null) {
            isHeadingAccurate = false;
            for (OdometryModule module : odometryModules) {
                if (module.getHeadingPriority() == highestHeadingPriority) {
                    highestPriorityHeading = module.getPosition();
                }
            }
        } else {
            isHeadingAccurate = true;
        }

        for (OdometryModule module : odometryModules) {
            if (module.doPositionResetToHigherPriority() && highestPriorityPosition != null &&
                    highestPositionPriorityModule != module) {
                Pose2D modulePosition = module.getPosition();
                module.setPosition(new Pose2D(DistanceUnit.INCH, highestPriorityPosition.getX(DistanceUnit.INCH),
                        highestPriorityPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES,
                        modulePosition.getHeading(AngleUnit.DEGREES)));
            }

            if (module.doHeadingResetToHigherPriority() && highestPriorityHeading != null && highestPositionPriorityModule != module) {
                Pose2D modulePosition = module.getPosition();
                module.setPosition(new Pose2D(DistanceUnit.INCH, modulePosition.getX(DistanceUnit.INCH),
                        modulePosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES,
                        highestPriorityHeading.getHeading(AngleUnit.DEGREES)));
            }
        }

        if (highestPriorityPosition != null && highestPriorityHeading != null) {
            position = new Pose2D(DistanceUnit.INCH, highestPriorityPosition.getX(DistanceUnit.INCH),
                    highestPriorityPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES,
                    highestPriorityHeading.getHeading(AngleUnit.DEGREES));
        }
    }

    // Behavior: Gets the current position taking into account all odometry sensors and their priorities.
    // Returns: A Pose2D containing the current position.
    public Pose2D getPosition() {
        return position;
    }

    // Behavior: Sets the position of all odometry modules to the given position.
    // Parameters:
    //      - Pose2D position: The position to set the position of the odometry modules to.
    public void setPosition(Pose2D position) {
        for (OdometryModule module : odometryModules) {
            module.setPosition(position);
        }
    }

    public void setPositionPriority(int priority) {
        positionPriority = priority;
    }

    public int getPositionPriority() {
        return positionPriority;
    }

    public void setHeadingPriority(int priority) {
        headingPriority = priority;
    }

    public int getHeadingPriority() {
        return headingPriority;
    }

    public void setDoPositionResetToHigherPriority(boolean doReset) {
        doPositionReset = doReset;
    }

    public boolean doPositionResetToHigherPriority() {
        return doPositionReset;
    }

    public void setDoHeadingResetToHigherPriority(boolean doReset) {
        doHeadingReset = doReset;
    }

    public boolean doHeadingResetToHigherPriority() {
        return doHeadingReset;
    }

    public boolean isPositionAccurate() {
        return isPositionAccurate;
    }

    public boolean isHeadingAccurate() {
        return isHeadingAccurate;
    }

    // Behavior: Resets all odometry modules positions.
    public void reset() {
        for (OdometryModule module : odometryModules) {
            module.reset();
        }
    }
}
