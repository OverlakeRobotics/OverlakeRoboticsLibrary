//

package org.firstinspires.ftc.teamcode.system;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BasicHolonomicDrivetrain {
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private int backLeftTarget;
    private int backRightTarget;
    private int frontLeftTarget;
    private int frontRightTarget;
    private double forward;
    private double strafe;
    private double turn;
    private DriveState currentDriveState;
    private enum DriveState {
        POSITION_DRIVE,
        POWER_DRIVE,
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

    // Behavior: Sets the power of the drive motors to the given powers.
    // Parameters:
    //      - double backLeftPower: The power for the back left motor.
    //      - double backRightPower: The power for the back right motor.
    //      - double frontLeftPower: The power for the front left motor.
    //      - double frontRightPower: The power for the front right motor.
    private void setPower(double backLeftPower, double backRightPower,
                          double frontLeftPower, double frontRightPower) {
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }

    // Behavior: Sets the mode of all the motors to the given mode.
    // Parameters:
    //      - DcMotorEx.RunMode mode: The mode to set the motors to.
    private void setMotorMode(DcMotorEx.RunMode mode) {
        backLeft.setMode(mode);
        backRight.setMode(mode);
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
    }

    // Behavior: Sets the drive power given a forward power, a strafe power, and a turn power. If
    //           power exceeds 1.0 (the limit) for any motor, it will scale all the powers down
    //           until the highest power is 1.0 to make sure the robot can still maneuver.
    // Parameters:
    //      - double forward: The forward power.
    //      - double strafe: The strafe power.
    //      - double turn: The turn power.
    private void moveRobot(double forward, double strafe, double turn) {
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                              Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        setPower(bl, br, fl, fr);
    }

    // Behavior: Drives the robot based on the current state and powers. This should be called
    //           every loop. The possible states are STOPPED, which sets the power to 0,
    //           POWER_DRIVE, which drives based on the current strafe, forward, and turn powers, and
    //           POSITION_DRIVE, which drives the motors to the current set position.
    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
                setPower(0, 0, 0, 0);

            case POWER_DRIVE:
                moveRobot(forward, strafe, turn);

            case POSITION_DRIVE:
                backLeft.setTargetPosition(backLeftTarget);
                backRight.setTargetPosition(backRightTarget);
                frontLeft.setTargetPosition(frontLeftTarget);
                frontRight.setTargetPosition(frontRightTarget);
                setPower(forward, forward, forward, forward);
        }
    }

    // Behavior: Sets the forward, strafe, and turn powers of the robot to the given values.
    // Parameters:
    //      - double forward: The given forward power.
    //      - double strafe: The given strafe power.
    //      - double turn: The given turn power.
    public void setPowerDrive(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        currentDriveState = DriveState.POWER_DRIVE;
    }

    // Behavior: Sets the position for all the motors and the power to drive to those positions.
    // Parameters:
    //      - int backLeftTarget: The target for the back left motor in counts.
    //      - int backRightTarget: The target for the back right motor in counts.
    //      - int frontLeftTarget: The target for the front left motor in counts.
    //      - int frontRightTarget: The target for the front right motor in counts.
    public void setPositionDrive(int backLeftTarget, int backRightTarget,
                                 int frontLeftTarget, int frontRightTarget, double power) {
        this.backLeftTarget = backLeftTarget;
        this.backRightTarget = backRightTarget;
        this.frontLeftTarget = frontLeftTarget;
        this.frontRightTarget = frontRightTarget;
        forward = power;
        strafe = 0;
        turn = 0;
        currentDriveState = DriveState.POSITION_DRIVE;
    }

    // Behavior: Stops the robot by setting the motor powers to 0.
    public void stop() {
        forward = 0;
        strafe = 0;
        turn = 0;
        currentDriveState = DriveState.STOPPED;
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The back left motor target position as an integer in counts.
    public int getBackLeftTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return backLeftTarget;
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The back right motor target position as an integer in counts.
    public int getBackRightTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return backRightTarget;
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The front left motor target position as an integer in counts.
    public int getFrontLeftTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return frontLeftTarget;
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The front right motor target position as an integer in counts.
    public int getFrontRightTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return frontRightTarget;
    }
}
