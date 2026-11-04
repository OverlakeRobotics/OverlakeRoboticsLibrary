package org.firstinspires.ftc.teamcode.code.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.parts.*;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


@Config
@TeleOp(name="Basic TeleOp", group="TeleOp")

public class BasicTeleOp extends OpMode {
    private OdometryHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    public double yOffset = -156;
    public double xOffset = -84.0;

    private Shooter pollenShooter;
    //private Shooter nectarShooter;

    public static double shooterVelocity = 0;
    public static double TICKS_PER_SEC = (double) 28 /60;

    private Pusher pusher;
    private Intake intake;
    private static double intakePower;

    private static Pose2D startPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        driveTrain.setPosition(startPos);

        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        //nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"));

        pusher = new Pusher(hardwareMap.get(Servo.class, "pusher"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();
        //telemetry.addData("Current Position: ", currentPos);

        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if (gamepad1.yWasPressed()) {
            shooterVelocity = 3000 * TICKS_PER_SEC;
            //shooterPower = 0.50;
        } else if (gamepad1.xWasPressed()) {
            shooterVelocity = 0;
            //shooterPower = 0.0;
        } else if (gamepad1.bWasPressed()) {
            //intakePower = 0.5;
            intakePower = 0.5;
        } else if (gamepad1.aWasPressed()) {
            intakePower = 0;
        } else if (gamepad1.leftBumperWasPressed()) {
            shooterVelocity -= 100 * TICKS_PER_SEC;
            //shooterPower -= 0.01;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterVelocity += 100 * TICKS_PER_SEC;
            //shooterPower += 0.01;
        }

        pusher.update(gamepad1.rightTriggerWasPressed());
        intake.setIntakePower(intakePower);

        pollenShooter.setShooterVelocity(shooterVelocity);
        //pollenShooter.setShooterPower(shooterPower);

        telemetry.addData("target rpm: ", shooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", pollenShooter.getShooterVelocity() * 60 / 28);
        //telemetry.addData("encoder position: ", pollenShooter.getCurrentPosition());
        //telemetry.addData("power: ", pollenShooter.getShooterPower());
        telemetry.update();
    }

}
