package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp(name="Intake Test", group="Test")

public class IntakeTest extends OpMode {
    private BasicHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Intake intake;
    public static double intakePower = 0;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
    }

    @Override
    public void loop() {
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if (gamepad1.yWasPressed()) {
            intakePower = 0.5;
        } else if (gamepad1.aWasPressed()) {
            intakePower = -0.5;
        } else if (gamepad1.xWasPressed()) {
            intakePower = 0;
        } else if (gamepad1.leftBumperWasPressed()) {
            intakePower -= 0.05;
        } else if (gamepad1.rightBumperWasPressed()) {
            intakePower += 0.05;
        }

        intake.setIntakePower(intakePower);

        telemetry.addData("Intake power", intakePower);
        telemetry.update();


        telemetry.addData("speed", intakePower);
        telemetry.update();

    }

}
