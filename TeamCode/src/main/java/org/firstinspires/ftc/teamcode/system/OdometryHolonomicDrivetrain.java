// Extension of the BasicHolonomicDrivetrain class adding the ability to use odometry. This adds
// features like heading correction, turning to a specific angle, field centric driving, moving to
// specific positions, and driving along a predefined path.

package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OdometryHolonomicDrivetrain extends BasicHolonomicDrivetrain {
    private static final double P_GAIN = 0.02;
    private static final double COUNTS_TO_INCHES = 0.03125;
    private static final boolean DO_STOPPED_HEADING_CORRECTION = true;
    private boolean doPositionHeadingCorrection;
    private final OdometryModule odometry;
    private Pose2D currentPosition;
    private Pose2D wantedPosition;

    public OdometryHolonomicDrivetrain(DcMotorEx backLeft, DcMotorEx backRight, DcMotorEx frontLeft,
                                       DcMotorEx frontRight, OdometryModule odometry) {
        super(backLeft, backRight, frontLeft, frontRight);
        this.odometry = odometry;
    }

    // Behavior: Overrides super classes drive method to include heading correction.
    @Override
    public void drive() {
        super.drive();
        switch (currentDriveState) {
            case STOPPED:
                if (DO_STOPPED_HEADING_CORRECTION) {
                    moveRobot(0, 0, getHeadingCorrection());
                }
                break;

            case POSITION_DRIVE:
                if (doPositionHeadingCorrection) {
                    moveRobot(forward, 0, getHeadingCorrection());
                }
                break;
        }
    }

    // Behavior: Overloads setPositionDrive to also include a wantedH for heading correction.
    public void setPositionDrive(int backLeftTarget, int backRightTarget, int frontLeftTarget,
                                 int frontRightTarget, double velocity, double wantedH) {
        super.setPositionDrive(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget, velocity);
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    // Behavior: Overloads setPositionDrive to also include a wantedH for heading correction.
    public void setPositionDrive(double forward, double strafe, double turn, double velocity, double wantedH) {
        super.setPositionDrive(forward, strafe, turn, velocity);
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    // Behavior: Overloads setPositionDrive to also include a wantedH for heading correction.
    public void setPositionDrive(int distance, double direction, double velocity, double wantedH) {
        super.setPositionDrive(distance, direction, velocity);
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    // Behavior: Overrides setPositionDrive from super to stop heading correction if a wantedH
    //           is not provided.
    @Override
    public void setPositionDrive(int backLeftTarget, int backRightTarget, int frontLeftTarget,
                                 int frontRightTarget, double velocity) {
        super.setPositionDrive(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget, velocity);
        doPositionHeadingCorrection = false;
    }

    // Behavior: Overrides setPositionDrive from super to stop heading correction if a wantedH
    //           is not provided.
    @Override
    public void setPositionDrive(double forward, double strafe, double turn, double velocity) {
        super.setPositionDrive(forward, strafe, turn, velocity);
        doPositionHeadingCorrection = false;
    }

    // Behavior: Overrides setPositionDrive from super to stop heading correction if a wantedH
    //           is not provided.
    @Override
    public void setPositionDrive(int distance, double direction, double velocity) {
        super.setPositionDrive(distance, direction, velocity);
        doPositionHeadingCorrection = false;
    }

    // Behavior: Overloaded method of setPositionDrive that sets the target position of the motors
    //           according to a given wanted position and a

    // Behavior: Sets the wanted heading of the robot.
    // Parameters:
    //      - double wantedHeading: The wanted heading of the robot in degrees.
    public void setWantedHeading(double wantedHeading) {
        wantedPosition = new Pose2D(DistanceUnit.INCH, wantedPosition.getX(DistanceUnit.INCH),
                wantedPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES, wantedHeading);
    }

    // Behavior: Overrides setDriveState to set the wanted position to the current position when stopped.
    @Override
    protected void setDriveState(DriveState driveState) {
        super.setDriveState(driveState);
        if (driveState == DriveState.STOPPED) {
            wantedPosition = currentPosition;
        }
    }

    // Behavior: Gets the turn velocity needed to heading correct the robot.
    // Returns: The turn velocity.
    private double getHeadingCorrection() {
        return P_GAIN * (wantedPosition.getHeading(AngleUnit.DEGREES) -
                         currentPosition.getHeading(AngleUnit.DEGREES));
    }

    public void updatePosition() {
        odometry.updatePosition();
        currentPosition = odometry.getPosition();
    }

    public Pose2D getPosition() {
        return currentPosition;
    }
}
