package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Thread-safe telemetry wrapper that buffers addData/addLine calls from any thread
 * and flushes them in insertion order when update() is called.
 * <p>
 * Every buffered entry is written to all of the sinks passed to the constructor, so
 * a dashboard's Telemetry can be added alongside the Driver Station one, e.g.
 * {@code new CustomTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())}.
 * <p>
 * Usage: call addData() from any thread at any time. Call update() from the
 * main loop thread to push all buffered entries to every sink.
 */
public class CustomTelemetry {
	private final Telemetry[] sinks;
	// caption == null marks a plain line
	private final ConcurrentLinkedQueue<String[]> buffer = new ConcurrentLinkedQueue<>();

	public CustomTelemetry(Telemetry... sinks) {
		if (sinks.length == 0) {
			throw new IllegalArgumentException("CustomTelemetry needs at least one Telemetry sink.");
		}
		this.sinks = sinks;
	}

	/**
	 * Thread-safe: buffers a telemetry entry. Can be called from any thread.
	 */
	public void addData(String caption, Object value) {
		buffer.add(new String[]{caption, String.valueOf(value)});
	}

	/**
	 * Thread-safe: buffers a plain text line. Can be called from any thread.
	 */
	public void addLine(String line) {
		buffer.add(new String[]{null, line});
	}

	public void clear() {
		buffer.clear();
	}

	/**
	 * Drains all buffered entries in insertion order to every sink,
	 * then pushes the update. Should be called from the main loop thread.
	 */
	public void update() {
		String[] entry;
		while ((entry = buffer.poll()) != null) {
			for (Telemetry sink : sinks) {
				if (entry[0] == null) {
					sink.addLine(entry[1]);
				} else {
					sink.addData(entry[0], entry[1]);
				}
			}
		}
		for (Telemetry sink : sinks) {
			sink.update();
		}
	}
}
