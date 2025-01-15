package org.firstinspires.ftc.teamcode.system;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface OdometryModule {
    public Pose2D getPosition();
    public void updatePosition();
    public void reset();
}
