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
    private boolean doPositionHeadingCorrection;
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
        Log.d("Heading Correction Test", "**DID HEADING CORRECTION 1**");
        Log.d("Heading Correction Test", "Current State: " + currentDriveState);
        Log.d("Heading Correction Test", "Do Position Heading Correction: " + doPositionHeadingCorrection);
        if (doPositionHeadingCorrection && currentDriveState == DriveState.POSITION_DRIVE) {
            Log.d("Heading Correction Test", "**DID HEADING CORRECTION 2**");
//            double correction = getHeadingCorrection();
//            Log.d("Heading Correction Test", "Correction Power: " + correction);
//            Log.d("Heading Correction Test", "Wanted Heading: " + wantedPosition.getHeading(AngleUnit.DEGREES));
//            Log.d("Heading Correction Test", "Current Heading: " + currentPosition.getHeading(AngleUnit.DEGREES));
//
//            // Test this tmrw
//            backLeftVelocity -= correction;
//            backRightVelocity += correction;
//            frontLeftVelocity -= correction;
//            frontRightVelocity += correction;
//
//            double min = Math.min(Math.min(backLeftVelocity, backRightVelocity),
//                                  Math.min(frontLeftVelocity, frontRightVelocity));
//
//            if (min < 0) {
//                min = Math.abs(min);
//                backLeftVelocity += min;
//                backRightVelocity += min;
//                frontLeftVelocity += min;
//                frontRightVelocity += min;
//            }
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
                super.setPositionDrive(getPositionDriveDistanceLeft(), positionDriveDirection - currentPosition.getHeading(AngleUnit.DEGREES), forward);
                super.drive();
                break;

            case VELOCITY_DRIVE:
                Log.d("Heading Correction Test", "BAD THINGS HAPPENED; STATE " + currentDriveState);
                wantedPosition = currentPosition;
                super.drive();
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
        super.setPositionDrive(distance, direction - currentPosition.getHeading(AngleUnit.DEGREES), velocity);
        positionDriveDirection = direction;
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
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

    public Pose2D getPosition() {
        return currentPosition;
    }
}
