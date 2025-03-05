package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

@Config
@TeleOp(name = "Basic OpMode", group = "TeleOp")
public class BasicOpMode extends OpMode {

    private BasicHolonomicDrivetrain driveTrain;
    private double lastTime;
    public static double velocity = 1000;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        driveTrain.setVelocityDrive(velocity, 0, 0);
    }

    @Override
    public void loop() {
        driveTrain.drive();
        double currentTime = runtime.seconds();
        double dt = currentTime - lastTime;
        telemetry.addData("Delta Time", dt);
        lastTime = currentTime;
    }
}
