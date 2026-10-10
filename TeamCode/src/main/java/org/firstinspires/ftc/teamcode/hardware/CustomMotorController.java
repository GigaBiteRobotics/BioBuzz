package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.controllers.PiecewiseController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.HashMap;

/**
 * CustomMotorController - Drives a group of {@link CustomMotor}s as one unit, either open-loop
 * (power) or closed-loop on the group's average RPM using a single shared Pedro {@link Controller}.
 */
public class CustomMotorController {
	/** What kind of controller is driving the group's RPM control. */
	public enum ControllerType {
		/** No controller set; {@link #updateRPMPID()} does nothing. */
		NONE,
		/** {@link Controller#zero}, which always outputs 0. */
		ZERO,
		/** A plain {@link Controller#pid} controller. */
		PID,
		/** A {@link Controller#piecewise} controller. */
		PIECEWISE,
		/** Anything else, e.g. a {@code plus}/{@code sum} composite or a feedforward term. */
		CUSTOM
	}

	private final String[] motorGroup;
	HashMap<String, CustomMotor> motors;
	boolean[] reverseMap;
	boolean[] encoderReverseMap;
	boolean[] encoderEnableMap;
	private volatile int targetRPM = 0;
	private volatile double targetPower = 0.0;
	private volatile boolean isRPMMode = false;

	// Single shared controller for group-level RPM control (Pedro convention: calculate(target, error))
	private volatile Controller groupController;
	private volatile ControllerType controllerType;
	private volatile double rpmTolerance = 10.0; // RPM
	private double lastRPMError = Double.NaN;
	// NaN = unknown (motors were last driven by setPower), so the next PID tick always writes
	private double lastAppliedGroupPower = Double.NaN;
	private boolean pidInitialized = false;

	// Lock object for thread safety
	private final Object lock = new Object();

	/**
	 * Constructor for motor group controller (all motors same encoder setting)
	 */
	public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean hasEncoder, double ticksPerRev, Controller controller) {
		this(hardwareMap, motorGroup, reverseMap, null, filled(motorGroup.length, hasEncoder), ticksPerRev, controller);
	}

	/**
	 * Constructor for motor group controller with encoder reverse map
	 */
	public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean[] encoderReverseMap, boolean hasEncoder, double ticksPerRev, Controller controller) {
		this(hardwareMap, motorGroup, reverseMap, encoderReverseMap, filled(motorGroup.length, hasEncoder), ticksPerRev, controller);
	}

	/**
	 * Constructor for motor group controller with per-motor encoder enable map
	 */
	public CustomMotorController(HardwareMap hardwareMap, String[] motorGroup, boolean[] reverseMap, boolean[] encoderReverseMap, boolean[] encoderEnableMap, double ticksPerRev, Controller controller) {
		this.motorGroup = motorGroup;
		this.reverseMap = reverseMap;
		this.encoderReverseMap = encoderReverseMap;
		this.encoderEnableMap = encoderEnableMap != null ? encoderEnableMap : new boolean[motorGroup.length];
		this.motors = new HashMap<>();
		this.groupController = controller;
		this.controllerType = typeOf(controller);

		if (motorGroup.length != reverseMap.length) {
			throw new IllegalArgumentException("Motor group and reverse map must have the same length.");
		}
		if (encoderReverseMap != null && motorGroup.length != encoderReverseMap.length) {
			throw new IllegalArgumentException("Motor group and encoder reverse map must have the same length.");
		}
		if (encoderEnableMap != null && motorGroup.length != encoderEnableMap.length) {
			throw new IllegalArgumentException("Motor group and encoder enable map must have the same length.");
		}

		for (int i = 0; i < motorGroup.length; i++) {
			String motorName = motorGroup[i];
			boolean reverseEnc = encoderReverseMap != null && encoderReverseMap[i];
			boolean hasEnc = this.encoderEnableMap[i];
			try {
				CustomMotor motor = new CustomMotor(hardwareMap, motorName, hasEnc, ticksPerRev, null, reverseEnc);
				this.motors.put(motorName, motor);
			} catch (Exception e) {
				throw new IllegalArgumentException("Failed to initialize motor: " + motorName + " - " + e.getMessage());
			}
		}
	}

	private static ControllerType typeOf(Controller controller) {
		if (controller == null) return ControllerType.NONE;
		if (controller == Controller.zero) return ControllerType.ZERO;
		if (controller instanceof PIDController) return ControllerType.PID;
		if (controller instanceof PiecewiseController) return ControllerType.PIECEWISE;
		return ControllerType.CUSTOM;
	}

	private static boolean[] filled(int length, boolean value) {
		boolean[] arr = new boolean[length];
		Arrays.fill(arr, value);
		return arr;
	}

	/**
	 * Set RPM for all motors in the group
	 */
	public void setRPM(int rpm) {
		synchronized (lock) {
			if (!isRPMMode) {
				// Reset PID state when switching into RPM mode
				if (groupController != null) groupController.reset();
				pidInitialized = false;
				lastAppliedGroupPower = Double.NaN;
			}
			if (!isRPMMode || this.targetRPM != rpm) {
				// The last error was measured against the old target, so don't report atTargetRPM from it
				lastRPMError = Double.NaN;
			}
			this.targetRPM = rpm;
			this.isRPMMode = true;
		}
	}

	/**
	 * Set power for all motors in the group
	 */
	public void setPower(double power) {
		synchronized (lock) {
			this.targetPower = power;
			this.isRPMMode = false;
			this.pidInitialized = false;
			this.lastRPMError = Double.NaN;
			this.lastAppliedGroupPower = Double.NaN;
			for (int i = 0; i < motorGroup.length; i++) {
				String motorName = motorGroup[i];
				CustomMotor motor = motors.get(motorName);
				if (motor != null) {
					double finalPower = reverseMap[i] ? -power : power;
					motor.setPower(finalPower);
				}
			}
		}
	}

	/**
	 * Update RPM PID for all motors in the group - call this in a loop.
	 * Uses Pedro's Controller: calculate(target, error), output clipped to [-1, 1].
	 * With no controller set, the group is driven at 0 power rather than left at its last output.
	 */
	public void updateRPMPID() {
		synchronized (lock) {
			if (isRPMMode) {
				double currentRPM = getAverageRPMInternal(true);
				double error = targetRPM - currentRPM;
				lastRPMError = error;

				Controller controller = groupController;
				double power = controller == null ? 0.0
						: Range.clip(controller.calculate(targetRPM, error), -1.0, 1.0);

				if (Double.isNaN(lastAppliedGroupPower)
						|| Math.abs(power - lastAppliedGroupPower) > 0.005
						|| (power == 0 && lastAppliedGroupPower != 0)
						|| (power != 0 && lastAppliedGroupPower == 0)) {
					for (int i = 0; i < motorGroup.length; i++) {
						String motorName = motorGroup[i];
						CustomMotor motor = motors.get(motorName);
						if (motor != null) {
							double finalPower = reverseMap[i] ? -power : power;
							motor.setRawPower(finalPower);
						}
					}
					lastAppliedGroupPower = power;
				}

				pidInitialized = true;
			}
		}
	}

	/**
	 * Average RPM over the encoder-enabled motors, in group direction (see {@link #getRPM()}).
	 *
	 * @param fresh true to force a hardware read, false to allow each motor's short-lived cache
	 */
	private double getAverageRPMInternal(boolean fresh) {
		double sum = 0.0;
		int count = 0;
		for (int i = 0; i < motorGroup.length; i++) {
			if (motors.get(motorGroup[i]).hasEncoder()) {
				sum += groupRPM(i, fresh);
				count++;
			}
		}
		return count > 0 ? sum / count : 0.0;
	}

	/**
	 * RPM of the motor at index {@code i}, flipped by {@link #reverseMap} so it reads in group direction.
	 */
	private double groupRPM(int i, boolean fresh) {
		CustomMotor motor = motors.get(motorGroup[i]);
		double rpm = fresh ? motor.updateAndGetRPM() : motor.getRPM();
		return reverseMap[i] ? -rpm : rpm;
	}

	/**
	 * Get average RPM of all motors in the group.
	 */
	public double getAverageRPM() {
		synchronized (lock) {
			double sum = 0.0;
			int count = 0;
			for (CustomMotor motor : motors.values()) {
				if (motor.hasEncoder()) {
					double rpm = motor.getRPM();
					sum += Math.abs(rpm);
					count++;
				}
			}
			return count > 0 ? sum / count : 0.0;
		}
	}

	/**
	 * Get the group's RPM: the signed average of every encoder-enabled motor, where positive is the
	 * direction a positive {@link #setPower(double)} / {@link #setRPM(int)} drives the group. This is
	 * the same value the RPM PID controls on. Returns 0 if no motor in the group has an encoder.
	 */
	public double getRPM() {
		return getAverageRPMInternal(false);
	}

	/**
	 * Get the RPM of a specific motor by name, signed in group direction like {@link #getRPM()}.
	 *
	 * @throws IllegalArgumentException if the motor isn't in this group
	 * @throws IllegalStateException    if the motor's encoder is disabled
	 */
	public double getRPM(String motorName) {
		int i = Arrays.asList(motorGroup).indexOf(motorName);
		if (i < 0) {
			throw new IllegalArgumentException("Motor not found in group: " + motorName
					+ " (group: " + Arrays.toString(motorGroup) + ")");
		}
		if (!motors.get(motorName).hasEncoder()) {
			throw new IllegalStateException("Encoder disabled for motor: " + motorName);
		}
		return groupRPM(i, false);
	}

	/**
	 * Set controller for group-level RPM control
	 */
	public void setController(Controller controller) {
		synchronized (lock) {
			// Reset so a previously used controller doesn't carry a stale integral/timestamp into this run
			if (controller != null) controller.reset();
			this.groupController = controller;
			this.controllerType = typeOf(controller);
			this.pidInitialized = false;
			this.lastRPMError = Double.NaN;
		}
	}

	/**
	 * Get the controller used for group-level RPM control. Returns the stored instance (no locking
	 * or rebuilding), so it's cheap to call every loop. May be null if none was provided.
	 */
	public Controller getController() {
		return groupController;
	}

	/**
	 * Get the kind of controller currently set. Worked out once in the constructor and
	 * {@link #setController(Controller)}, so it's free to check every loop.
	 */
	public ControllerType getControllerType() {
		return controllerType;
	}

	/**
	 * Set how close (in RPM) the group must be to the target for {@link #atTargetRPM()}.
	 */
	public void setRPMTolerance(double rpmTolerance) {
		this.rpmTolerance = rpmTolerance;
	}

	public int getTargetRPM() {
		return targetRPM;
	}

	public double getTargetPower() {
		return targetPower;
	}

	/**
	 * Get the last applied PID output power
	 */
	public double getPIDOutput() {
		synchronized (lock) {
			return pidInitialized ? lastAppliedGroupPower : 0.0;
		}
	}

	/**
	 * @return whether the last PID update was within tolerance of the target RPM
	 */
	public boolean atTargetRPM() {
		synchronized (lock) {
			return groupController != null && Math.abs(lastRPMError) <= rpmTolerance;
		}
	}

	public boolean isRPMMode() {
		return isRPMMode;
	}

	/**
	 * Get a debug string with the raw encoder data behind each motor's RPM: which hub port it reads,
	 * the encoder position (should change while the motor spins; stuck = unplugged/wrong port),
	 * the raw ticks/s from the hub, and the RPM computed from it.
	 */
	public String getDebugString() {
		StringBuilder sb = new StringBuilder();
		for (String motorName : motorGroup) {
			CustomMotor motor = motors.get(motorName);
			if (motor != null && motor.hasEncoder()) {
				DcMotorEx dc = motor.getMotor();
				double rawVel = motor.getRawVelocity();
				double ticksPerRev = motor.getTicksPerRev();
				double expectedRPM = (ticksPerRev > 0) ? (rawVel / ticksPerRev) * 60.0 : 0;
				sb.append(String.format("%s [%s port %d]: pos=%d vel=%.0f t/s rpm=%.1f tpr=%.1f | ",
						motorName, dc.getController().getConnectionInfo(), dc.getPortNumber(),
						dc.getCurrentPosition(), rawVel, expectedRPM, ticksPerRev));
			}
		}
		return sb.toString();
	}
}
