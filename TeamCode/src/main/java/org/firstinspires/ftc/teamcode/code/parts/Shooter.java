package org.firstinspires.ftc.teamcode.code.parts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    public final double MAX_VELOCITY = 2800;
    private final DcMotorEx shooterMotor;

    public Shooter (DcMotorEx shooterMotor) {
        this.shooterMotor = shooterMotor;

        this.shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setShooterVelocity(double velocity) {
        double clamped = Range.clip(velocity, 0, MAX_VELOCITY);
        this.shooterMotor.setVelocity(clamped);
    }

    public double getShooterVelocity() {
        return shooterMotor.getVelocity();
    }

    public void setShooterPower(double power) {
        double clamped = Range.clip(power, -1.0, 1.0);
        this.shooterMotor.setPower(clamped);
    }

    public double getShooterPower() {
        return shooterMotor.getPower();
    }

    public double getCurrentPosition() {
        return shooterMotor.getCurrentPosition();
    }
}
