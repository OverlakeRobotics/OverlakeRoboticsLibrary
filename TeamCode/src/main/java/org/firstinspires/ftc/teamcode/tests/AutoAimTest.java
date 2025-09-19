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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

    public double goalX = 60;
    public double goalY = 54;

    // Positive angle is to the left, positive x is forward, and positive y is left
    // This is the center of the bot when the program is initialized
    public Pose2D startPos = new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 0);
    public Pose2D[] presetPositions = {
            new Pose2D(DistanceUnit.INCH, -54, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 27, 21, AngleUnit.DEGREES, 0),
    };

    public int currentPreset = -1;

    public double velocity = 2000;

    private OdometryHolonomicDrivetrain driveTrain;
    private static final ElapsedTime runtime = new ElapsedTime();

    private static final int targetID = 24;

    private boolean autoLock = false;

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
        driveTrain.setPosition(startPos);

        runtime.reset();
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();

        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult targetApril = null;
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : aprilTags) {
                Pose3D robotPose = tag.getRobotPoseFieldSpace();
                Position pos = robotPose.getPosition();
                telemetry.addData("April Tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", tag.getFiducialId(), tag.getFamily(), tag.getTargetXDegrees(), tag.getTargetYDegrees());
                telemetry.addData("Tag Robot Pose", "X: %.2f, Y: %.2f, Z: %.2f, H: %.2f", pos.x, pos.y, pos.z, robotPose.getOrientation().getYaw());

                if (tag.getFiducialId() == targetID) {
                    targetApril = tag;
                    break;
                }
            }
        }

        double wantedHeading;
        if (targetApril != null) {
            wantedHeading = driveTrain.getPosition().getHeading(AngleUnit.DEGREES) - targetApril.getTargetXDegrees();
        } else {
            Pose2D pos = driveTrain.getPosition();
            wantedHeading = Math.atan2(
                    goalY - pos.getY(DistanceUnit.INCH),
                    goalX - pos.getX(DistanceUnit.INCH)
            );
        }

        if (gamepad1.a) {
            currentPreset = 0;
        } else if (gamepad1.b) {
            currentPreset = 1;
        }

        if (currentPreset >= 0 && (Math.abs(gamepad1.left_stick_x) > 0.001 || Math.abs(gamepad1.left_stick_y) > 0.001 || Math.abs(gamepad1.right_stick_x) > 0.001)) {
            currentPreset = -1;
            autoLock = true;
        }

        if (currentPreset >= 0) {
            Pose2D wantedPosition = new Pose2D(
                DistanceUnit.INCH,
                presetPositions[currentPreset].getX(DistanceUnit.INCH),
                presetPositions[currentPreset].getX(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                wantedHeading
            );
            driveTrain.setPositionDrive(wantedPosition, velocity);
        } else {
            double turn = -gamepad1.right_stick_x * velocity;

            if (gamepad1.y) {
                autoLock = true;
            }

            if (Math.abs(turn) > 0.1) {
                autoLock = false;
            }

            if (autoLock) {
                driveTrain.setWantedHeading(wantedHeading);
                turn = driveTrain.getHeadingCorrectionVelocity();
            }

            driveTrain.setVelocityDriveFieldCentric(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, turn);
        }

        telemetry.update();
        driveTrain.drive();
    }
}
