package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.HubDevices;
import org.firstinspires.ftc.teamcode.hardware.HubIMU;

public class Constants {
    // TODO: match these names to the robot's hardware config, then confirm directions with the Mecanum Tuner
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("lf");
                c.backLeftName.set("lr");
                c.frontRightName.set("rf");
                c.backRightName.set("rr");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    // Odometry pods plug into hub encoder ports and the heading comes from the hub's embedded IMU.
    // Neither needs to be in the Driver Station configuration; HubDevices builds them from these ports.
    // TODO: set to the hub and motor ports the forward (X) and strafe (Y) pods are plugged into
    public static final HubDevices.Hub ODOMETRY_HUB = HubDevices.Hub.CONTROL;
    public static final int X_POD_PORT = 0;
    public static final int Y_POD_PORT = 3;

    // goBILDA 4-bar pod: 2000 ticks per rev on a 32 mm wheel
    private static final double GOBILDA_4_BAR_INCHES_PER_TICK = Math.PI * 32.0 / 25.4 / 2000.0;

    // TODO: replace offsets/directions/scalars with the Two Wheel Tuner output, and set the hub's mounting orientation
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(
            c -> {
                c.imu.set(new HubIMU(HubDevices.Hub.CONTROL, new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
                c.imuName.set(HubDevices.imuName(HubDevices.Hub.CONTROL)); // not looked up; HubIMU builds the IMU itself

                c.xPodName.set(HubDevices.encoderName(ODOMETRY_HUB, X_POD_PORT));
                c.yPodName.set(HubDevices.encoderName(ODOMETRY_HUB, Y_POD_PORT));

                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);
                c.xPodDirection.set(Encoder.FORWARD);
                c.yPodDirection.set(Encoder.FORWARD);
                c.forwardTicksToInches.set(GOBILDA_4_BAR_INCHES_PER_TICK);
                c.strafeTicksToInches.set(GOBILDA_4_BAR_INCHES_PER_TICK);
            }
    );

    // Placeholder values from the upstream example robot so Foresight can be constructed.
    // Manual driving doesn't use them; run the Foresight Tuner before following paths or holding.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
                c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));

                c.headingFeedback.set(Controller.proportional(5.258721785960744));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

                c.maxAchievableForwardVelocity.set(72.72923108818539);
                c.maxAchievableStrafeVelocity.set(52.34323936525474);
                c.naturalForwardDeceleration.set(85.01144677379789);
                c.naturalStrafeDeceleration.set(104.49787535782846);
            }
    );

    public static Follower create(HardwareMap h) {
        HubDevices.registerEncoder(h, ODOMETRY_HUB, X_POD_PORT);
        HubDevices.registerEncoder(h, ODOMETRY_HUB, Y_POD_PORT);
        return new Follower(new TwoWheelLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
