// Extension of the BasicHolonomicDrivetrain class adding the ability to use odometry. This adds
// features like heading correction, turning to a specific angle, field centric driving, moving to
// specific positions, and driving along a predefined path.

package org.firstinspires.ftc.teamcode.system;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OdometryHolonomicDrivetrain extends BasicHolonomicDrivetrain {
    private static final double P_GAIN = 50;
    private static final boolean DO_STOPPED_HEADING_CORRECTION = true;
    private static final double MIN_DIST_TO_STOP = 0.5;
    private boolean doPositionHeadingCorrection;
    private boolean positionDriveUsingOdometry;
    private final OdometryModule odometry;
    private Pose2D currentPosition;
    private Pose2D wantedPosition;
    private double positionDriveDirection;

    public OdometryHolonomicDrivetrain(DcMotorEx backLeft, DcMotorEx backRight, DcMotorEx frontLeft,
                                       DcMotorEx frontRight, OdometryModule odometry) {
        super(backLeft, backRight, frontLeft, frontRight);
        this.odometry = odometry;
        this.currentPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        this.wantedPosition = currentPosition;
    }

    @Override
    public void setVelocity(double backLeftVelocity, double backRightVelocity,
                            double frontLeftVelocity, double frontRightVelocity) {
        if (doPositionHeadingCorrection && currentDriveState == DriveState.POSITION_DRIVE) {
            double correction = getHeadingCorrection();
            backLeftVelocity -= correction;
            backRightVelocity += correction;
            frontLeftVelocity -= correction;
            frontRightVelocity += correction;
        }
        super.setVelocity(backLeftVelocity, backRightVelocity, frontLeftVelocity, frontRightVelocity);
    }

    // Behavior: Overrides super classes drive method to include heading correction.
    @Override
    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
                if (DO_STOPPED_HEADING_CORRECTION) {
                    moveRobot(0, 0, getHeadingCorrection());
                    break;
                }
                super.drive();
                break;

            case POSITION_DRIVE:
                if (positionDriveUsingOdometry) {
                    setPositionDrive(wantedPosition, forward);
                    if (getDistanceToDestination() < MIN_DIST_TO_STOP) {
                        setDriveState(DriveState.STOPPED);
                    }
                } else {
                    super.setPositionDrive(getPositionDriveDistanceLeft(), positionDriveDirection - currentPosition.getHeading(AngleUnit.DEGREES), forward);
                }
                super.drive();
                break;

            case VELOCITY_DRIVE:
                wantedPosition = currentPosition;
                super.drive();
                break;
        }
    }

    // Behavior: Overloads setPositionDrive to also include a wantedH for heading correction.
    public void setPositionDrive(int backLeftTarget, int backRightTarget, int frontLeftTarget,
                                 int frontRightTarget, double velocity, double wantedH) {
        setPositionDrive(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget, velocity);
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    @Override
    public void setPositionDrive(int backLeftTarget, int backRightTarget, int frontLeftTarget, int frontRightTarget, double velocity) {
        super.setPositionDrive(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget, velocity);
        doPositionHeadingCorrection = true;
        positionDriveUsingOdometry = false;
    }

    // Behavior: Overloads setPositionDrive to also include a wantedH for heading correction.
    public void setPositionDrive(double forward, double strafe, double turn, double velocity, double wantedH) {
        super.setPositionDrive(forward, strafe, turn, velocity);
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    // Behavior: Overloads setPositionDrive to also include a wantedH for heading correction.
    public void setPositionDrive(int distance, double direction, double velocity, double wantedH) {
        super.setPositionDrive(distance, direction - currentPosition.getHeading(AngleUnit.DEGREES), velocity);
        positionDriveDirection = direction;
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    public void setPositionDrive(Pose2D wantedPosition, double velocity) {
        this.wantedPosition = wantedPosition;
        forward = velocity;
        double dx = this.wantedPosition.getX(DistanceUnit.INCH) - this.currentPosition.getX(DistanceUnit.INCH);
        double dy = this.wantedPosition.getY(DistanceUnit.INCH) - this.currentPosition.getY(DistanceUnit.INCH);
        int counts = (int) Math.round(Math.hypot(dx * FORWARD_COUNTS_PER_INCH, dy * FORWARD_COUNTS_PER_INCH));
        double direction = Math.toDegrees(Math.atan2(dx, dy));
        setPositionDrive(counts, direction, velocity, wantedPosition.getHeading(AngleUnit.DEGREES));
        positionDriveUsingOdometry = true;
    }

    public double getDistanceToDestination() {
        return Math.hypot(Math.pow(wantedPosition.getX(DistanceUnit.INCH) - currentPosition.getX(DistanceUnit.INCH), 2),
                          Math.pow(wantedPosition.getY(DistanceUnit.INCH) - currentPosition.getY(DistanceUnit.INCH), 2));
    }

    // Behavior: Sets the wanted heading of the robot.
    // Parameters:
    //      - double wantedHeading: The wanted heading of the robot in degrees.
    public void setWantedHeading(double wantedHeading) {
        wantedPosition = new Pose2D(DistanceUnit.INCH, wantedPosition.getX(DistanceUnit.INCH),
                wantedPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES, wantedHeading);
    }

    // Behavior: Gets the turn velocity needed to heading correct the robot.
    // Returns: The turn velocity.
    private double getHeadingCorrection() {
        return P_GAIN * normalize(normalize(wantedPosition.getHeading(AngleUnit.DEGREES)) -
                         normalize(currentPosition.getHeading(AngleUnit.DEGREES)));
    }

    private static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    public void updatePosition() {
        odometry.updatePosition();
        currentPosition = odometry.getPosition();
    }

    public boolean isStopped() {
        return currentDriveState == DriveState.STOPPED;
    }

    public Pose2D getPosition() {
        return currentPosition;
    }
}
