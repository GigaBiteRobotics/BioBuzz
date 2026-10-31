package org.firstinspires.ftc.teamcode.contants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MDOConstants {
    public static double IntakeSpeedIN = 1.0;
    public static double IntakeSpeedOUT = -1.0;
    public static double BlockServoHigh = 0.00;
    public static double BlockServoLow = -0.35;
    public static int LauncherRPM = 2000;
    // Launcher velocity control, error in RPM: power = KF * targetRPM + KP * error + KI * ∫error dt
    // Read live every loop, so dashboard edits apply immediately
    public static double LauncherKF = 0.0002; // ~1 / free speed of a bare 6000 RPM motor; tune this first
    public static double LauncherKP = 0.001; // 200 RPM error -> +0.1 power
    public static double LauncherKI = 0.001;
    public static double LauncherIZone = 100; // RPM; the integral only builds within this band, so no spin-up windup
    public static double LauncherMaxI = 100; // cap on ∫error (RPM·s), so KI can add at most KI * MaxI power

}
