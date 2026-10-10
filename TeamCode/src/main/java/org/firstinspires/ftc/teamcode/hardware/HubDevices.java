package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * HubDevices - Builds encoder and IMU devices straight from REV hub ports, so they
 * don't need to be added to the Driver Station robot configuration.
 *
 * <p>Built devices are registered into the {@link HardwareMap} under generated names
 * (see {@link #encoderName}) so code that looks devices up by name, like Pedro's
 * localizers, can use them. The HardwareMap outlives a single OpMode, so devices
 * from an earlier OpMode are reused instead of rebuilt.</p>
 */
public class HubDevices {
	/** The hub's embedded IMU lives on I2C bus 0. */
	private static final int EMBEDDED_IMU_BUS = 0;

	public enum Hub {
		/** The hub the Driver Station connects to (the Control Hub, or a USB-connected Expansion Hub). */
		CONTROL,
		/** An Expansion Hub connected over RS-485. */
		EXPANSION
	}

	/**
	 * Find a hub in the active configuration.
	 *
	 * @throws IllegalStateException if that hub isn't connected
	 */
	public static LynxModule getHub(HardwareMap hardwareMap, Hub hub) {
		for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
			if (module.isParent() == (hub == Hub.CONTROL)) {
				return module;
			}
		}
		throw new IllegalStateException("No " + hub + " hub found in the active configuration.");
	}

	public static String encoderName(Hub hub, int port) {
		return hub + "_encoder" + port;
	}

	public static String imuName(Hub hub) {
		return hub + "_imu";
	}

	/**
	 * Register the encoder on a motor port (0-3) and return the name it was registered under.
	 * The port may also drive a motor; reading its encoder doesn't affect that.
	 */
	public static String registerEncoder(HardwareMap hardwareMap, Hub hub, int port) {
		if (port < 0 || port > 3) {
			throw new IllegalArgumentException("Motor port must be 0-3, got " + port);
		}
		String name = encoderName(hub, port);
		if (hardwareMap.tryGet(DcMotorEx.class, name) == null) {
			hardwareMap.put(name, new DcMotorImplEx(getMotorController(hardwareMap, getHub(hardwareMap, hub)), port));
		}
		return name;
	}

	/**
	 * Get the hub's embedded IMU, building it on first use. The caller still has to
	 * {@link IMU#initialize(IMU.Parameters)} it with the hub's mounting orientation.
	 */
	public static IMU getImu(HardwareMap hardwareMap, Hub hub) {
		String name = imuName(hub);
		IMU imu = hardwareMap.tryGet(IMU.class, name);
		if (imu != null) {
			return imu;
		}

		LynxModule module = getHub(hardwareMap, hub);
		LynxI2cDeviceSynch i2c = LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, module, EMBEDDED_IMU_BUS);
		switch (module.getImuType()) {
			case BHI260:
				imu = new BHI260IMU(i2c, true);
				break;
			case BNO055:
				imu = new LynxEmbeddedBNO055IMUNew(i2c, true);
				break;
			default:
				i2c.close();
				throw new IllegalStateException("The " + hub + " hub has no usable embedded IMU (type " + module.getImuType() + ").");
		}
		hardwareMap.put(name, imu);
		return imu;
	}

	/**
	 * Reuse the motor controller the SDK built for this hub, so its motor state isn't reset.
	 */
	private static DcMotorController getMotorController(HardwareMap hardwareMap, LynxModule module) {
		for (LynxDcMotorController controller : hardwareMap.getAll(LynxDcMotorController.class)) {
			if (controller.getSerialNumber().equals(module.getSerialNumber())) {
				return controller;
			}
		}
		try {
			return new LynxDcMotorController(hardwareMap.appContext, module);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			throw new IllegalStateException("Interrupted while creating motor controller", e);
		} catch (Exception e) {
			throw new IllegalStateException("Could not create motor controller for hub " + module.getSerialNumber(), e);
		}
	}
}
