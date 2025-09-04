package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.components.IMUOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "TeleOp Example", group = "TeleOp")
public class TeleOpExample extends OpMode {
    public static final double MAX_VELOCITY = 1500;
    private BasicHolonomicDrivetrain driveTrain;
    private double lastTime;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        IMU gyro = hardwareMap.get(IMU.class, "imu");
        gyro.resetYaw();
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );
    }

    @Override
    public void loop() {
        // Set velocity targets based on gamepad input
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * MAX_VELOCITY,
                -gamepad1.left_stick_x * MAX_VELOCITY,
                -gamepad1.right_stick_x * MAX_VELOCITY);
        // Make the motors reach target velocities
        driveTrain.drive();
    }
}
