package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Mapping Test", group = "Test")
public class MotorMapping extends OpMode {
    private static final double TEST_POWER = 0.2;

    private DcMotorEx shooter;

    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontLeft;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        for (DcMotorEx motor : new DcMotorEx[]{frontRight, backLeft, backRight, frontLeft, shooter}) {
            motor.setPower(0);
            motor.setDirection(DcMotorEx.Direction.FORWARD);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        frontRight.setPower(gamepad1.y ? TEST_POWER : 0);
        backLeft.setPower(gamepad1.b ? TEST_POWER : 0);
        backRight.setPower(gamepad1.a ? TEST_POWER : 0);
        frontLeft.setPower(gamepad1.x ? TEST_POWER : 0);
        shooter.setPower(gamepad1.right_bumper ? TEST_POWER : 0);

        telemetry.addData("Y - frontRight", frontRight.getPower());
        telemetry.addData("B - backLeft", backLeft.getPower());
        telemetry.addData("A - backRight", backRight.getPower());
        telemetry.addData("X - frontLeft", frontLeft.getPower());
        telemetry.addData("RB - shooter", shooter.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : new DcMotorEx[]{frontRight, backLeft, backRight, frontLeft, shooter}) {
            motor.setPower(0);
        }
    }
}
