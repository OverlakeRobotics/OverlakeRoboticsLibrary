package org.firstinspires.ftc.teamcode.code.helpers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Holds the last autonomous pose for the next TeleOp in the same Robot Controller session. */
public final class AutoPoseHandoff {
    private static Pose2D savedPose;

    private AutoPoseHandoff() { }

    public static void save(Pose2D pose) {
        savedPose = new Pose2D(
                DistanceUnit.INCH,
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                pose.getHeading(AngleUnit.DEGREES)
        );
    }

    /** Returns and clears the saved pose so it cannot be reused by a later unrelated TeleOp. */
    public static Pose2D consume() {
        Pose2D pose = savedPose;
        savedPose = null;
        return pose;
    }
}
