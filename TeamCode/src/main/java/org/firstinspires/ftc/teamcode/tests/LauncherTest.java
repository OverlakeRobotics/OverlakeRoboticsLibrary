package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

@Config
@TeleOp(name = "Launcher Test", group = "TeleOp")
public class LauncherTest extends OpMode {
    private DcMotorEx cwLeftMotor;
    private DcMotorEx ccwRightMotor;
    @Override
    public void init() {
        cwLeftMotor = hardwareMap.get(DcMotorEx.class, "cwLeftMotor");
        cwLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        cwLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ccwRightMotor = hardwareMap.get(DcMotorEx.class, "ccwRightMotor");
        ccwRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        cwLeftMotor.setPower(gamepad1.left_trigger);
        ccwRightMotor.setPower(gamepad1.right_trigger);
    }
}
