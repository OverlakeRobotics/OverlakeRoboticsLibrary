package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Pedro Pathing robot constants: the one file to fill in for your robot. Run the tuners in
// Tuning.java through AutoTune (http://192.168.43.1:10158 while connected to the robot) and paste
// the generated configs here. Using something other than a Pinpoint or Mecanum? Change create().
public class Constants {
    // TODO: set to true once all three tuners are done. Until then the planner is not sent the speed
    // limits below (they are not your robot's) and OpModes show a warning in telemetry.
    public static final boolean PEDRO_TUNED = false;

    // Same names and directions as BasicHolonomicDrivetrain. TODO: confirm with the Mecanum tuner.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("backLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    // TODO: put the offsets from the Pinpoint tuner here. Every OpMode that uses Pedro or
    // PathPlanLegacy reads them from here, so this is the only copy. Pod directions go below.
    public static final double X_POD_OFFSET_INCHES = 0;
    public static final double Y_POD_OFFSET_INCHES = 0;

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.offsetUnits.set(DistanceUnit.INCH);
                c.xPodOffset.set(X_POD_OFFSET_INCHES);
                c.yPodOffset.set(Y_POD_OFFSET_INCHES);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
            }
    );

    // TODO: replace with the output of the Foresight tuner. These numbers are from another robot and
    // are only a starting point so the code runs; your robot will not follow paths well until tuned.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3839311031683409);
                Controller secondaryTranslationalForward = Controller.proportional(0.14185228379268575);
                Controller primaryTranslationalLateral = Controller.proportional(0.5818701636724144);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21498547761983483);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01280597892601108));
                c.brake.set(Controller.proportionalFeedforward(0.010885082087109418));

                c.headingFeedback.set(Controller.proportional(7.222633278655314));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05855290463912122, 0.0047427233096400696));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09680435641764054, 0.048774997795879585));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016249787489419178, 0.0021564408959034256));

                c.maxAchievableForwardVelocity.set(76.97089871346982);
                c.maxAchievableStrafeVelocity.set(57.37803304172739);
                c.naturalForwardDeceleration.set(28.184084156498187);
                c.naturalStrafeDeceleration.set(62.665075412593474);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
