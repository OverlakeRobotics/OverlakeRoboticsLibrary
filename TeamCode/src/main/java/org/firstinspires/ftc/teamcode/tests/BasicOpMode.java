package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

@TeleOp(name = "Basic OpMode", group = "TeleOp")
public class BasicOpMode extends OpMode {

    private BasicHolonomicDrivetrain driveTrain;

    @Override
    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        driveTrain.setPositionDrive(1000, 45, 1000);
    }

    @Override
    public void loop() {
        driveTrain.drive();
    }
}
