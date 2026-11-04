package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.code.parts.*;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@TeleOp(name="Shooter Test", group="Test")

public class ShooterTest extends OpMode {
    private BasicHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Shooter pollenShooter;
    //private Shooter nectarShooter;

    public static double shooterVelocity = 0;
    public static double TICKS_PER_SEC = (double) 28 /60;
    public static double shooterPower = 0;


    @Override
    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        //nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"));

    }

    @Override
    public void loop() {
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if (gamepad1.yWasPressed()) {
            shooterVelocity = 100 * TICKS_PER_SEC;
        } else if (gamepad1.xWasPressed()) {
            shooterVelocity = 0;
        } else if (gamepad1.leftBumperWasPressed()) {
            shooterVelocity -= 50 * TICKS_PER_SEC;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterVelocity += 50 * TICKS_PER_SEC;

        } else if (gamepad1.bWasPressed()) {
            shooterPower = 0.25;
        } else if (gamepad1.aWasPressed()) {
            shooterPower = 0.0;
        } else if (gamepad1.rightTriggerWasPressed()) {
            shooterPower += 0.1;
        } else if (gamepad1.leftTriggerWasPressed()) {
            shooterPower -= 0.1;
        }

        pollenShooter.setShooterVelocity(shooterVelocity);
        //pollenShooter.setShooterPower(shooterPower);

        telemetry.addData("target rpm: ", shooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", pollenShooter.getShooterVelocity() * 60 / 28);
        telemetry.addData("encoder position: ", pollenShooter.getCurrentPosition());
        telemetry.addData("power: ", pollenShooter.getShooterPower());
        telemetry.update();
    }

}
