package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainDriveThreads.DriveThread;
import org.firstinspires.ftc.teamcode.contants.MDOConstants;
import org.firstinspires.ftc.teamcode.hardware.CustomMotorController;
import org.firstinspires.ftc.teamcode.hardware.CustomServoController;
import org.firstinspires.ftc.teamcode.hardware.HubInitializer;
import org.firstinspires.ftc.teamcode.input.GamepadEventHandler;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.telemetry.CustomTelemetry;

import java.util.Arrays;


@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {

    // gp1handler reads gamepad1 (driver), gp2handler reads gamepad2 (operator)
    private GamepadEventHandler gp1handler, gp2handler;
    private CustomTelemetry telemetryC;
    private Follower follower;
    // Runs follower.manual()/update() off the OpMode thread; don't touch the follower directly while it's running
    private DriveThread driveThread;

    enum IntakeStateList {
        IN,
        OUT,
        STOP
    }
    IntakeStateList intakeState = IntakeStateList.STOP;
    enum IntakeBlockerStateList {
        DOWN,
        UP
    }
    IntakeBlockerStateList intakeBlockerState = IntakeBlockerStateList.DOWN;
    CustomMotorController intakeMotor, launcherMotor;
    CustomServoController blockerServo;
    boolean launcherState = false;
    // Gains the launcher PID was last built from, so dashboard edits are picked up without rebuilding every loop
    private double[] appliedLauncherPID;


    @Override
    public void init() {
        telemetryC = new CustomTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HubInitializer.initBulkCaching(hardwareMap);

        follower = Constants.create(hardwareMap);
        driveThread = new DriveThread(follower);

        gp1handler = new GamepadEventHandler();
        gp2handler = new GamepadEventHandler();

        // ===== Gamepad 2 (operator) =====
        // Bumpers toggle the intake; pressing the other bumper switches direction
        gp2handler.onPress("intakeIn", gp -> gp.right_bumper, () -> intakeState =
                intakeState == IntakeStateList.IN ? IntakeStateList.STOP : IntakeStateList.IN);
        gp2handler.onPress("intakeOut", gp -> gp.left_bumper, () -> intakeState =
                intakeState == IntakeStateList.OUT ? IntakeStateList.STOP : IntakeStateList.OUT);
        gp2handler.onPress("blockerToggle", gp -> gp.b, () -> intakeBlockerState =
                intakeBlockerState == IntakeBlockerStateList.UP ? IntakeBlockerStateList.DOWN : IntakeBlockerStateList.UP);
        gp2handler.onPress("launcherToggle", gp -> gp.y, () -> launcherState = !launcherState);

        intakeMotor = new CustomMotorController(
                hardwareMap,
                new String[]{"intake"},
                new boolean[]{false},
                false,
                28.0,
                Controller.pid(0,0,0)
        );
        // Plain position servo: no analog feedback, so the PIDF gains are unused
        blockerServo = new CustomServoController(
                hardwareMap,
                new String[]{"blocker"},
                new boolean[]{false},
                false,
                new double[]{0, 0, 0, 0},
                null
        );
        launcherMotor = new CustomMotorController(
                hardwareMap,
                new String[]{"launcher"},
                new boolean[]{true},
                true,
                28.0,
                Controller.pid(0, 0,0)
        );

        telemetryC.addLine("Initialized");
        telemetryC.update();
    }

    @Override
    public void start() {
        follower.setPose(Pose.zero());
        driveThread.start();
    }

    @Override
    public void loop() {
        HubInitializer.clearBulkCache();

        gp1handler.update(gamepad1);
        gp2handler.update(gamepad2);

        driveThread.checkHealth();
        // Robot-centric: Pedro uses +strafe = left and +turn = counter-clockwise, so the stick X axes are negated
        driveThread.setDriveInput(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        CheckLoop();

        Pose pose = driveThread.getPose();
        telemetryC.addData("X", pose.x());
        telemetryC.addData("Y", pose.y());
        telemetryC.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetryC.addData("Drive Loop (ms)", driveThread.getLoopTimeMs());
        telemetryC.addData("Intake", intakeState);
        telemetryC.addData("Blocker", intakeBlockerState);
        telemetryC.addData("Launcher", launcherState ? "ON" : "OFF");
        telemetryC.addData("Launcher Target RPM", launcherMotor.getTargetRPM());
        telemetryC.addData("Launcher Motor RPM", launcherMotor.getRPM());
        telemetryC.addData("Launcher Ready", launcherMotor.atTargetRPM());
        telemetryC.addData("Launcher Encoder", launcherMotor.getDebugString());
        telemetryC.addData("LauncherState", launcherState);
        telemetryC.addData("BlockerState", intakeBlockerState);
        telemetryC.update();
    }

    @Override
    public void stop() {
        // Join the drive thread first so it can't overwrite the stop with another manual() call
        driveThread.stop();
        follower.stop();
        follower.drivetrain.stop();
        intakeMotor.setPower(0);
        launcherMotor.setPower(0);
    }

    public void CheckLoop() {
        switch (intakeState) {
            case IN:
                intakeMotor.setPower(MDOConstants.IntakeSpeedIN);
                break;
            case OUT:
                intakeMotor.setPower(MDOConstants.IntakeSpeedOUT);
                break;
            case STOP:
                intakeMotor.setPower(0);
        }
        switch (intakeBlockerState) {
            // CustomServoController maps [-1, 1] onto the servo's [0, 1] range
            case UP:
                blockerServo.setPosition(MDOConstants.BlockServoHigh);
                break;
            case DOWN:
                blockerServo.setPosition(MDOConstants.BlockServoLow);
                break;
        }
        if (launcherState) {
            UpdateLauncherPID();
            launcherMotor.setRPM(MDOConstants.LauncherMotorRPM);
            launcherMotor.updateRPMPID();
        } else {
            // setPower leaves RPM mode, so the PID stops driving the motor
            launcherMotor.setPower(0);
        }
    }

    /**
     * Rebuild the launcher PID only when it isn't a PID yet or the dashboard gains changed, so the
     * controller's integral/derivative state carries over between loops.
     */
    public void UpdateLauncherPID() {
        double[] pid = MDOConstants.LauncherPID;
        if (launcherMotor.getControllerType() == CustomMotorController.ControllerType.PID
                && Arrays.equals(pid, appliedLauncherPID)) {
            return;
        }
        appliedLauncherPID = pid.clone();
        launcherMotor.setController(Controller.pid(pid[0], pid[1], pid[2]));
    }
}
