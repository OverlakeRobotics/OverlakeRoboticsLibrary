package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Pusher {
    public static double HOME_DEGREES = -100.0; //-65
    public static double PUSH_DEGREES = -5.0; //5
    public static double TRAVEL_MS = 500.0;

    private final Servo servo;
    private final ElapsedTime timer = new ElapsedTime();

    private int state = 0;
    private boolean previousPressed = false;

    public Pusher(Servo servo) {
        this.servo = servo;
        servo.setPosition((HOME_DEGREES + 150.0) / 300.0);
    }

    public Pusher(Servo servo, double homeDegrees, double pushDegrees, double travelMs) {
        this.servo = servo;
        this.HOME_DEGREES = homeDegrees;
        this.PUSH_DEGREES = pushDegrees;
        this.TRAVEL_MS = travelMs;
        servo.setPosition((HOME_DEGREES + 150.0) / 300.0);
    }

    public void update(boolean pressed) {
        if (state == 0 && pressed && !previousPressed) {
            servo.setPosition((PUSH_DEGREES + 150.0) / 300.0);
            timer.reset();
            state = 1;
        } else if (state == 1 && timer.milliseconds() >= TRAVEL_MS) {
            servo.setPosition((HOME_DEGREES + 150.0) / 300.0);
            timer.reset();
            state = 2;
        } else if (state == 2 && timer.milliseconds() >= TRAVEL_MS) {
            state = 0;
        }

        previousPressed = pressed;
    }

    public int getState() {
        return state;
    }

    public void setPosition(double degrees) {
        servo.setPosition((degrees + 150.0) / 300.0);
    }

    public void home() {
        servo.setPosition((HOME_DEGREES + 150.0) / 300.0);
        state = 0;
    }
}