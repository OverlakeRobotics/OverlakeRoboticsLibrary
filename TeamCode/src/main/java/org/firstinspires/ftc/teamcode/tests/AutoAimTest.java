package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

import java.util.List;

@Config
@TeleOp(name = "Auto Aim Test", group = "TeleOp")
public class AutoAimTest extends OpMode {
    private Limelight3A limelight;

    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm

    private OdometryHolonomicDrivetrain driveTrain;

    private static final int targetID = 1;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : aprilTags) {
                telemetry.addData("April Tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", tag.getFiducialId(), tag.getFamily(), tag.getTargetXDegrees(), tag.getTargetYDegrees());

                if (tag.getFiducialId() == targetID) {
                    // Check whether its + or - tag.getTargetXDegrees() (or maybe YDegrees)
                    driveTrain.setWantedHeading(driveTrain.getPosition().getHeading(AngleUnit.DEGREES) + tag.getTargetXDegrees());
                }
            }
        }

        driveTrain.setVelocityDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, driveTrain.getHeadingCorrectionVelocity());

        driveTrain.drive();
    }
}
