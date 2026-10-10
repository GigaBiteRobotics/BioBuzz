package org.firstinspires.ftc.teamcode.MainDriveThreads;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.hardware.HubInitializer;

/**
 * DriveThread - Runs Pedro's {@link Follower} (manual drive + localization) on its own thread, so
 * drive updates aren't held back by everything else in the TeleOp loop.
 *
 * <p><b>Threading rules:</b></p>
 * <ul>
 *   <li>While running, only this thread touches the Follower. The OpMode passes stick input in
 *       with {@link #setDriveInput} and reads the pose back with {@link #getPose()}.</li>
 *   <li>Don't read {@code gamepad1} from this thread: the SDK copies into it on the OpMode thread
 *       before each {@code loop()}, so reads from here can see a half-updated gamepad.</li>
 *   <li>Each iteration clears the hub bulk cache so odometry pod reads are fresh
 *       ({@code LynxModule.clearBulkCache()} is synchronized, so both threads may clear it).</li>
 *   <li>If the loop throws, the drivetrain is stopped and the exception is rethrown on the OpMode
 *       thread by {@link #checkHealth()}, instead of the thread dying silently.</li>
 * </ul>
 */
public class DriveThread {
	// Long enough for an in-flight follower.update() to finish; well under the SDK's stop() stuck-detection limit
	private static final long STOP_TIMEOUT_MS = 250;

	/** Stick input, held in one immutable object so forward/strafe/turn are always read as a set. */
	private static final class DriveInput {
		final double forward, strafe, turn;

		DriveInput(double forward, double strafe, double turn) {
			this.forward = forward;
			this.strafe = strafe;
			this.turn = turn;
		}
	}

	private final Follower follower;
	private Thread driveThread;
	private volatile boolean driveThreadRunning = false;

	private volatile DriveInput input = new DriveInput(0, 0, 0);
	private volatile Pose pose = Pose.zero();
	private volatile double loopTimeMs = 0.0;
	private volatile RuntimeException failure;

	public DriveThread(Follower follower) {
		this.follower = follower;
	}

	/**
	 * Start the drive loop. Set the follower's starting pose before calling this.
	 */
	public synchronized void start() {
		if (driveThreadRunning) return;
		failure = null;
		pose = follower.pose();
		driveThreadRunning = true;
		driveThread = new Thread(this::run, "DriveThread");
		driveThread.start();
	}

	/**
	 * Stop the drive loop and wait for it to finish, so the caller can safely use the Follower again.
	 */
	public synchronized void stop() {
		driveThreadRunning = false;
		Thread t = driveThread;
		if (t == null) return;
		try {
			// Let the current iteration finish instead of interrupting it mid hardware transaction
			t.join(STOP_TIMEOUT_MS);
			if (t.isAlive()) {
				t.interrupt();
				t.join(STOP_TIMEOUT_MS);
			}
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
		driveThread = null;
	}

	/**
	 * Set the robot-centric drive input, in Pedro's convention (+strafe = left, +turn = counter-clockwise).
	 */
	public void setDriveInput(double forward, double strafe, double turn) {
		input = new DriveInput(forward, strafe, turn);
	}

	/**
	 * @return the pose from the drive thread's most recent update
	 */
	public Pose getPose() {
		return pose;
	}

	/**
	 * @return how long the last drive loop iteration took, in milliseconds
	 */
	public double getLoopTimeMs() {
		return loopTimeMs;
	}

	public boolean isRunning() {
		return driveThreadRunning;
	}

	/**
	 * Rethrow any exception the drive loop died with. Call once per OpMode loop so a crash shows up
	 * on the Driver Station instead of the robot silently losing drive control.
	 */
	public void checkHealth() {
		RuntimeException e = failure;
		if (e != null) {
			throw new RuntimeException("Drive thread crashed: " + e.getMessage(), e);
		}
	}

	private void run() {
		long lastTime = System.nanoTime();
		try {
			while (driveThreadRunning && !Thread.currentThread().isInterrupted()) {
				HubInitializer.clearBulkCache();

				DriveInput in = input;
				follower.manual(in.forward, in.strafe, in.turn);
				follower.update();
				pose = follower.pose();

				long now = System.nanoTime();
				loopTimeMs = (now - lastTime) / 1e6;
				lastTime = now;
			}
		} catch (RuntimeException e) {
			failure = e;
		} finally {
			driveThreadRunning = false;
			try {
				follower.drivetrain.stop();
			} catch (RuntimeException e) {
				if (failure == null) failure = e;
			}
		}
	}
}
