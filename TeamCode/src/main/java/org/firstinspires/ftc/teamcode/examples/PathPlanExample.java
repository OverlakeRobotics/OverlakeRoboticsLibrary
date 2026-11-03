package org.firstinspires.ftc.teamcode.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.PathPlanAuto;


// This is an example OpMode designed to interface with the Overlake Robotics Path Planner.
// It lets you upload a path from the planner to the robot and follows it with Pedro Pathing. Fill
// in pedro/Constants.java for your robot first. Two tags are handled for you:
//      - velocity: Caps the robot's speed. The value is a percent of max speed (100 = full speed).
//      - pause: Stops the robot for a number of seconds equal to the tag's value.
// Everything else your robot does goes in the list in tags() below. Copy this OpMode and replace
// the list with your own tags.
@Disabled
@Autonomous(name = "Path Planning Example", group = "Autonomous")
public class PathPlanExample extends PathPlanAuto {
    private DcMotorEx intake;   // null when the robot has no "intake" motor

    @Override
    protected void setup() {
        intake = hardwareMap.tryGet(DcMotorEx.class, "intake");
        if (intake != null) intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    protected void tags() {
        // Runs the intake at the tag's value in ticks/s (0 stops it). The robot keeps driving.
        if (intake != null) tag("intake", velocity -> intake.setVelocity(velocity));
    }

    @Override
    protected void finish() {
        if (intake != null) intake.setVelocity(0);
    }
}
