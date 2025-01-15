package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OdometryHolonomicDrivetrain extends BasicHolonomicDrivetrain {
    private final OdometryCollection odometry;

    public OdometryHolonomicDrivetrain(DcMotorEx backLeft, DcMotorEx backRight, DcMotorEx frontLeft,
                                       DcMotorEx frontRight, OdometryCollection odometry) {
        super(backLeft, backRight, frontLeft, frontRight);
        this.odometry = odometry;
    }

    public void updatePosition() {
        odometry.updatePosition();
    }

    public Pose2D getPosition() {
        return odometry.getPosition();
    }
}
