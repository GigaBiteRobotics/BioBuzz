package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.revhub.localizers.CustomIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * HubIMU - Pedro {@link CustomIMU} backed by a hub's embedded IMU, built through
 * {@link HubDevices} so it doesn't need to be in the robot configuration.
 * The hardware map name Pedro passes to {@link #initialize} is ignored.
 */
public class HubIMU implements CustomIMU {
	private final HubDevices.Hub hub;
	private final RevHubOrientationOnRobot hubOrientation;
	private IMU imu;

	public HubIMU(HubDevices.Hub hub, RevHubOrientationOnRobot hubOrientation) {
		this.hub = hub;
		this.hubOrientation = hubOrientation;
	}

	@Override
	public void initialize(HardwareMap hardwareMap, String hardwareMapName) {
		imu = HubDevices.getImu(hardwareMap, hub);
		imu.initialize(new IMU.Parameters(hubOrientation));
	}

	@Override
	public double getHeading() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
	}

	@Override
	public void resetYaw() {
		imu.resetYaw();
	}
}
