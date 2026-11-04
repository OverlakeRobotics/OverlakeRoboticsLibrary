package org.firstinspires.ftc.teamcode.code.parts;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private final DcMotorEx intakeMotor;

    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;

        this.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setIntakePower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        intakeMotor.setPower(power);
    }
}