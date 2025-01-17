// This class contains basic movement for robots with holonomic movement (Mecanum or Omni)
// It includes functions that set velocity to the motors, set target positions to the motors, and
// stop the robot.

package org.firstinspires.ftc.teamcode.system;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BasicHolonomicDrivetrain {
    public static final double MAX_STOP_VELOCITY = 1e-2;
    public static final int MAX_VELOCITY = 3000;
    public static final double FORWARD_TO_STRAFE_RATIO = 1; // 1.19148;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    protected double forward;
    protected double strafe;
    protected double turn;
    protected DriveState currentDriveState;
    public enum DriveState {
        POSITION_DRIVE,
        VELOCITY_DRIVE,
        STOPPED

    };

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
        backLeft.setVelocity(backLeftVelocity);
        backRight.setVelocity(backRightVelocity);
        frontLeft.setVelocity(frontLeftVelocity);
        frontRight.setVelocity(frontRightVelocity);
    }

    // Behavior: Sets the mode of all the motors to the given mode.
    // Parameters:
    //      - DcMotorEx.RunMode mode: The mode to set the motors to.
    protected void setMotorMode(DcMotorEx.RunMode mode) {
        backLeft.setMode(mode);
        backRight.setMode(mode);
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
    }

    // Behavior: Sets the drive velocity given a forward velocity, a strafe velocity, and a turn
    //           velocity. If velocity exceeds 1.0 (the limit) for any motor, it will scale all the
    //           velocities down until the highest velocity is 1.0 to make sure the robot can still
    //           maneuver.
    // Parameters:
    //      - double forward: The forward velocity.
    //      - double strafe: The strafe velocity.
    //      - double turn: The turn velocity.
    protected void moveRobot(double forward, double strafe, double turn) {
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                              Math.max(Math.abs(bl), Math.abs(br)));
        if (max > MAX_VELOCITY) {
            fl *= MAX_VELOCITY / max;
            fr *= MAX_VELOCITY / max;
            bl *= MAX_VELOCITY / max;
            br *= MAX_VELOCITY / max;
        }

        setVelocity(bl, br, fl, fr);
    }

    // Behavior: Drives the robot based on the current state and velocities. This should be called
    //           every loop. The possible states are STOPPED, which sets the velocity to 0,
    //           VELOCITY_DRIVE, which drives based on the current strafe, forward, and turn velocities,
    //           and POSITION_DRIVE, which drives the motors to the current set position.
    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
                setVelocity(0, 0, 0, 0);
                break;

            case VELOCITY_DRIVE:
                if (Math.abs(forward) <= MAX_STOP_VELOCITY && Math.abs(strafe) <= MAX_STOP_VELOCITY &&
                                                           Math.abs(turn) <= MAX_STOP_VELOCITY) {
                    setDriveState(DriveState.STOPPED);
                }
                moveRobot(forward, strafe, turn);
                break;

            case POSITION_DRIVE:
                setVelocity(forward, forward, forward, forward);
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
    //           based on given forward, strafe, and turn count values.
    // Parameters:
    //      - double forward: The number of forward counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double strafe: The number of strafe counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double turn: The number of turn counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double velocity: The velocity to move the robot at.
    public void setPositionDrive(double forward, double strafe, double turn, double velocity) {
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int)(forward - strafe + turn));
        backRight.setTargetPosition(backRight.getTargetPosition() + (int)(forward + strafe - turn));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int)(forward + strafe + turn));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + (int)(forward - strafe - turn));
        this.forward = velocity;
        this.strafe = 0;
        this.turn = 0;
        setDriveState(DriveState.POSITION_DRIVE);
    }

    // Behavior: An overloaded method of setPositionDrive that sets the motor targets based on
    //           a direction (relative to the robot), and a distance (in counts).
    // Parameters:
    //      - int distance: The distance to drive, in counts.
    //      - double direction: The direction relative to the robot to drive in, in degrees.
    //                          Forward is 0 degrees, left is positive, right is negative.
    //      - double velocity: The velocity to move the robot at.
    public void setPositionDrive(int distance, double direction, double velocity) {
        double forwardCounts = distance * Math.cos(Math.toRadians(direction));
        double strafeCounts = distance * Math.sin(Math.toRadians(direction)) * FORWARD_TO_STRAFE_RATIO;
        Log.d("Counts", "ForwardCounts: " + forwardCounts);
        Log.d("Counts", "StrafeCounts: " + strafeCounts);
        setPositionDrive(forwardCounts, strafeCounts, 0, velocity);
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
                setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
        currentDriveState = driveState;
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
}
