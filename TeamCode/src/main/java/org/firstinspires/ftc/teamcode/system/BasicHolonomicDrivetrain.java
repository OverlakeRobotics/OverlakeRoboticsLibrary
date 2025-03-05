// This class contains basic movement for robots with holonomic movement (Mecanum or Omni)
// It includes functions that set velocity to the motors, set target positions to the motors, and
// stop the robot.

package org.firstinspires.ftc.teamcode.system;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class BasicHolonomicDrivetrain {
    public static final double MAX_STOP_VELOCITY = 1e-2;
    public static final int MAX_VELOCITY = 3000;
    public static double FORWARD_TO_STRAFE_RATIO = /* 1.0822; */ 1.19148;
    public static double FORWARD_COUNTS_PER_INCH = 30;
    public static double COUNTS_TO_SLOW_DOWN = 500;
    public static double MIN_POSITION_VELOCITY = 200;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    protected double forward;
    protected double strafe;
    protected double turn;
    protected int countsAffectedByTurn;

    protected DriveState currentDriveState;
    public enum DriveState {
        POSITION_DRIVE,
        VELOCITY_DRIVE,
        STOPPED

    }

    public BasicHolonomicDrivetrain(DcMotorEx backLeft, DcMotorEx backRight,
                                    DcMotorEx frontLeft, DcMotorEx frontRight) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        currentDriveState = DriveState.STOPPED;
    }

    // Behavior: Sets the velocity of the drive motors to the given velocities.
    // Parameters:
    //      - double backLeftVelocity: The velocity for the back left motor.
    //      - double backRightVelocity: The velocity for the back right motor.
    //      - double frontLeftVelocity: The velocity for the front left motor.
    //      - double frontRightVelocity: The velocity for the front right motor.
    protected void setVelocity(double backLeftVelocity, double backRightVelocity,
                               double frontLeftVelocity, double frontRightVelocity) {
        double max = Math.max(Math.max(Math.abs(frontLeftVelocity), Math.abs(frontRightVelocity)),
                Math.max(Math.abs(backLeftVelocity), Math.abs(backRightVelocity)));

        if (max > MAX_VELOCITY) {
            frontLeftVelocity *= MAX_VELOCITY / max;
            frontRightVelocity *= MAX_VELOCITY / max;
            backLeftVelocity *= MAX_VELOCITY / max;
            backRightVelocity *= MAX_VELOCITY / max;
        }

        backLeft.setVelocity(backLeftVelocity);
        backRight.setVelocity(backRightVelocity);
        frontLeft.setVelocity(frontLeftVelocity);
        frontRight.setVelocity(frontRightVelocity);
    }

    // Behavior: Sets the mode of all the motors to the given mode.
    // Parameters:
    //      - DcMotorEx.RunMode mode: The mode to set the motors to.
    protected void setMotorMode(DcMotorEx.RunMode mode) {
        if (backLeft.getMode() != mode) {
            backLeft.setMode(mode);
            backRight.setMode(mode);
            frontLeft.setMode(mode);
            frontRight.setMode(mode);
        }
    }

    // Behavior: Sets the drive velocity given a forward velocity, a strafe velocity, and a turn
    //           velocity. If velocity exceeds MAX_VELOCITY, it will scale all the velocities
    //           down until the highest velocity is MAX_VELOCITY to make sure the robot can still
    //           maneuver. The directions follow the unit circle.
    // Parameters:
    //      - double forward: The forward velocity. Positive for forwards and negative for backwards.
    //      - double strafe: The strafe velocity. Positive for left and negative for right.
    //      - double turn: The turn velocity. Positive for left and negative for right.
    protected void moveRobot(double forward, double strafe, double turn) {
        double bl = forward + strafe - turn;
        double br = forward - strafe + turn;
        double fl = forward - strafe - turn;
        double fr = forward + strafe + turn;

        setVelocity(bl, br, fl, fr);
    }

    // Behavior: Drives the robot based on the current state and velocities. This should be called
    //           every loop. The possible states are STOPPED, which sets the velocity to 0,
    //           VELOCITY_DRIVE, which drives based on the current strafe, forward, and turn velocities,
    //           and POSITION_DRIVE, which drives the motors to the current set position.
    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
                setPositionDrive(getBackLeftPosition(), getBackRightPosition(), getFrontLeftPosition(), getFrontRightPosition(), 1000);
                break;

            case VELOCITY_DRIVE:
                if (Math.abs(forward) <= MAX_STOP_VELOCITY && Math.abs(strafe) <= MAX_STOP_VELOCITY &&
                                                           Math.abs(turn) <= MAX_STOP_VELOCITY) {
                    setDriveState(DriveState.STOPPED);
                }
                moveRobot(forward, strafe, turn);
                break;

            case POSITION_DRIVE:
                if (!isDriving()) {
                    setDriveState(DriveState.STOPPED);
                    break;
                }
                double backLeftDif = (getBackLeftTarget() - getBackLeftPosition());
                double backRightDif = (getBackRightTarget() - getBackRightPosition());
                double frontLeftDif = (getFrontLeftTarget() - getFrontLeftPosition());
                double frontRightDif = (getFrontRightTarget() - getFrontRightPosition());

                double max = Math.max(Math.max(Math.abs(backLeftDif), Math.abs(backRightDif)),
                        Math.max(Math.abs(frontLeftDif), Math.abs(frontRightDif)));
                backLeftDif /= max;
                backRightDif /= max;
                frontLeftDif /= max;
                frontRightDif /= max;

                double countsLeft = getPositionDriveDistanceLeft();
                double velocity = forward;
                if (countsLeft < COUNTS_TO_SLOW_DOWN) {
                    velocity = MIN_POSITION_VELOCITY + (forward - MIN_POSITION_VELOCITY) * (countsLeft / COUNTS_TO_SLOW_DOWN);
                }

                Log.d("Velocity", "Velocity: " + velocity);
                Log.d("Velocity", "Forward : " + forward);

                setVelocity(velocity * backLeftDif, velocity * backRightDif,
                        velocity * frontLeftDif, velocity * frontRightDif);
                break;
        }
    }

    // Behavior: Sets the forward, strafe, and turn velocities of the robot to the given values.
    // Parameters:
    //      - double forward: The given forward velocity.
    //      - double strafe: The given strafe velocity.
    //      - double turn: The given turn velocity.
    public void setVelocityDrive(double forward, double strafe, double turn) {
        if (Math.abs(forward) <= MAX_STOP_VELOCITY && Math.abs(strafe) <= MAX_STOP_VELOCITY &&
                Math.abs(turn) <= MAX_STOP_VELOCITY) {
            setDriveState(DriveState.STOPPED);
            return;
        }
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        setDriveState(DriveState.VELOCITY_DRIVE);
    }

    // Behavior: Sets the position for all the motors and the velocity to drive to those positions.
    // Parameters:
    //      - int backLeftTarget: The target for the back left motor in counts.
    //      - int backRightTarget: The target for the back right motor in counts.
    //      - int frontLeftTarget: The target for the front left motor in counts.
    //      - int frontRightTarget: The target for the front right motor in counts.
    //      - double velocity: The velocity to move the robot at.
    public void setPositionDrive(int backLeftTarget, int backRightTarget,
                                 int frontLeftTarget, int frontRightTarget, double velocity) {
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        forward = velocity;
        strafe = 0;
        turn = 0;
        setDriveState(DriveState.POSITION_DRIVE);
    }

    // Behavior: An overloaded method of setPositionDrive that sets the position of the motors
    //           based on given forward, strafe, and turn count values. Directions follow the
    //           unit circle.
    // Parameters:
    //      - double forward: The number of forward counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double strafe: The number of strafe counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double turn: The number of turn counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double velocity: The velocity to move the robot at.
    public void setPositionDrive(double forward, double strafe, double turn, double velocity) {
        setPositionDrive(backLeft.getCurrentPosition() + (int)(forward + strafe - turn),
                backRight.getCurrentPosition() + (int)(forward - strafe + turn),
                frontLeft.getCurrentPosition() + (int)(forward - strafe - turn),
                frontRight.getCurrentPosition() + (int)(forward + strafe + turn), velocity);
        countsAffectedByTurn = (int)turn;
        Log.d("Testing Counts", "Did the thing: " + countsAffectedByTurn);
    }

    // Behavior: An overloaded method of setPositionDrive that sets the motor targets based on
    //           a direction (relative to the robot), a distance (in counts), and a turn (in counts).
    // Parameters:
    //      - int distance: The distance to drive, in counts.
    //      - double direction: The direction relative to the robot to drive in, in degrees.
    //                          Forward is 0 degrees, left is positive, right is negative.
    //      - double velocity: The velocity to move the robot at.
    public void setPositionDrive(int distance, double direction, double turn, double velocity) {
        double forwardCounts = distance * Math.cos(Math.toRadians(direction));
        double strafeCounts = distance * Math.sin(Math.toRadians(direction)) * FORWARD_TO_STRAFE_RATIO;
        setPositionDrive(forwardCounts, strafeCounts, turn, velocity);
    }

    public void setPositionDrive(int distance, double direction, double velocity) {
        setPositionDrive(distance, direction, 0, velocity);
    }

    // Behavior: Gets the number of counts the robot needs to move forward to finish the current
    //           position drive.
    // Returns: The number of forward counts remaining in the current position drive as an int.
    public int getForwardCountsLeft() {
        if (currentDriveState == DriveState.POSITION_DRIVE) {
            return ((getBackLeftTarget() + countsAffectedByTurn - getBackLeftPosition()) +
                    (getBackRightTarget() - countsAffectedByTurn - getBackRightPosition()) +
                    (getFrontLeftTarget() + countsAffectedByTurn - getFrontLeftPosition()) +
                    (getFrontRightTarget() - countsAffectedByTurn - getFrontRightPosition())) / 4;
        }
        return 0;
    }

    // Behavior: Gets the number of counts the robot needs to strafe to finish the current
    //           position drive.
    // Returns: The number of strafe counts remaining in the current position drive as an int.
    public int getStrafeCountsLeft() {
        if (currentDriveState == DriveState.POSITION_DRIVE) {
            return ((getBackLeftTarget() + countsAffectedByTurn - getBackLeftPosition()) -
                    (getBackRightTarget() - countsAffectedByTurn - getBackRightPosition()) -
                    (getFrontLeftTarget() + countsAffectedByTurn - getFrontLeftPosition()) +
                    (getFrontRightTarget() - countsAffectedByTurn - getFrontRightPosition())) / 4;
        }
        return 0;
    }

    // Behavior: Gets the number of counts the robot needs to turn to finish the current
    //           position drive.
    // Returns: The number of turn counts remaining in the current position drive as an int.
    public int getTurnCountsLeft() {
        if (currentDriveState == DriveState.POSITION_DRIVE) {
            return ((getBackLeftTarget() - getBackLeftPosition()) -
                    (getBackRightTarget() - getBackRightPosition()) +
                    (getFrontLeftTarget() - getFrontLeftPosition()) -
                    (getFrontRightTarget() - getFrontRightPosition())) / 4;
        }
        return 0;
    }

    // Behavior: Gets the distance from the destination left in the current position drive, in
    //           counts.
    // Returns: A double containing the distance to the destination in counts.
    public int getPositionDriveDistanceLeft() {
        return (int)Math.hypot(getForwardCountsLeft(), getStrafeCountsLeft() / FORWARD_TO_STRAFE_RATIO);
    }

    // Behavior: Gets the direction of the current position drive.
    // Returns: A double, containing the direction of the current position drive in degrees.
    public double getPositionDriveDirection() {
        return Math.toDegrees(Math.atan2(getStrafeCountsLeft() / FORWARD_TO_STRAFE_RATIO, getForwardCountsLeft()));
    }

    // Behavior: Stops the robot by setting the motor velocities to 0.
    public void stop() {
        forward = 0;
        strafe = 0;
        turn = 0;
        setDriveState(DriveState.STOPPED);
    }

    // Behavior: Sets the drive state of the robot.
    // Parameters:
    //      - DriveState driveState: The drive state to set the robot to.
    protected void setDriveState(DriveState driveState) {
        switch (driveState) {
            case POSITION_DRIVE:
                setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case VELOCITY_DRIVE:
            case STOPPED:
                setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
        currentDriveState = driveState;
    }

    // Behavior: Returns whether the robot is currently driving
    // Returns: A boolean that is true if the robot is currently driving, false otherwise.
    public boolean isDriving() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }

    // Behavior: Gets the current drive state of the robot.
    // Returns: A DriveState object containing the current drive state.
    protected DriveState getDriveState(){
        return currentDriveState;
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The back left motor target position as an integer in counts.
    public int getBackLeftTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return backLeft.getTargetPosition();
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The back right motor target position as an integer in counts.
    public int getBackRightTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return backRight.getTargetPosition();
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The front left motor target position as an integer in counts.
    public int getFrontLeftTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return frontLeft.getTargetPosition();
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The front right motor target position as an integer in counts.
    public int getFrontRightTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return frontRight.getTargetPosition();
    }

    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }

    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }
}
