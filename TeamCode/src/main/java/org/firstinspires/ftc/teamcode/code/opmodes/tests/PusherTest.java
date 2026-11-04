package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.code.parts.Pusher;

@TeleOp(name = "Pusher Test", group = "Test")
public class PusherTest extends OpMode {
    private Pusher pusher;
    public static double HOME_DEGREES = -100.0;
    public static double PUSH_DEGREES = -5.0;
    public static double TRAVEL_MS = 500.0;
    public static double servoPosition = -100.0;

    @Override
    public void init() {
        Servo servo = hardwareMap.get(Servo.class, "pusher");
        pusher = new Pusher(servo, HOME_DEGREES, PUSH_DEGREES, TRAVEL_MS);
    }

    @Override
    public void loop() {
        pusher.update(gamepad1.rightTriggerWasPressed());

        if (gamepad1.yWasPressed()) {
            servoPosition += 5.0;
            pusher.setPosition(servoPosition);
        } else if (gamepad1.aWasPressed()) {
            servoPosition -= 5.0;
            pusher.setPosition(servoPosition);
        }
        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("Pusher State", pusher.getState());

    }


}