package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

/**
 * Flywheel PID tuner. Runs the real FlywheelSubsystem so what you tune is exactly what the match code uses.
 * Nudge kV / kP / kI from the gamepad (or from FTC Dashboard), and watch target vs actual RPM on the
 * Dashboard graph (add "target", "actual", "error" to the graph).
 *
 * Gamepad1:
 *   X            = flywheels on/off (turning on = spin-up test from 0)
 *   Y            = while running, jump between HIGH and LOW rpm (step / recovery test)
 *   dpad up/down = HIGH rpm +/- 100
 *   LB / RB      = select gain (kV -> kP -> kI)
 *   dpad left/right = selected gain down/up
 *   A            = flip flywheel direction (if they spin backwards)
 *
 * Tuned values only live until the app restarts, so paste the printed line into FlywheelSubsystem.
 * Your remembered shoot RPM is restored when this opmode ends.
 */
@TeleOp(name = "FlywheelTuner", group = "Test")
public class FlywheelTuner extends LinearOpMode {
    private static final double START_HIGH_RPM = 3000;
    private static final double LOW_RPM = 1500;
    private static final double RPM_STEP = 100;
    private static final double MAX_RPM = 6000;

    private static final String[] GAIN_NAMES = {"kV", "kP", "kI"};
    private static final double[] GAIN_STEPS = {0.01, 0.00005, 0.0001};

    // step-response metrics, reset each time the setpoint changes
    private long stepNanos;
    private double stepDir = 1;       // +1 stepping up, -1 stepping down
    private double overshoot = 0;     // worst RPM past the setpoint
    private double settleSeconds = Double.NaN;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FlywheelSubsystem flywheel = new FlywheelSubsystem(hardwareMap);
        double originalRpm = flywheel.getTargetRpm();

        double highRpm = START_HIGH_RPM;
        boolean atHigh = true;
        boolean running = false;
        int selected = 0;
        boolean lastX = false, lastY = false, lastA = false, lastLb = false, lastRb = false;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        flywheel.setRpm(highRpm);

        telemetry.addLine("Ready. X = start/stop flywheels.");
        telemetry.update();

        waitForStart();
        try {
            while (opModeIsActive()) {
                flywheel.update();

                double setpoint = atHigh ? highRpm : LOW_RPM;
                double actual = flywheel.getActualRpm();

                // ---- controls ----
                if (gamepad1.x && !lastX) {
                    running = !running;
                    flywheel.setRpm(setpoint);
                    flywheel.setRunning(running);
                    if (running) startStep(0, setpoint);
                }
                if (gamepad1.y && !lastY && running) {
                    atHigh = !atHigh;
                    double newSetpoint = atHigh ? highRpm : LOW_RPM;
                    startStep(setpoint, newSetpoint);
                    flywheel.setRpm(newSetpoint);
                }
                if (gamepad1.a && !lastA) flywheel.flipDirection();
                if ((gamepad1.dpad_up && !lastUp) || (gamepad1.dpad_down && !lastDown)) {
                    highRpm = Math.max(0, Math.min(MAX_RPM, highRpm + (gamepad1.dpad_up ? RPM_STEP : -RPM_STEP)));
                    if (atHigh) {
                        startStep(setpoint, highRpm);
                        flywheel.setRpm(highRpm);
                    }
                }
                if (gamepad1.right_bumper && !lastRb) selected = (selected + 1) % GAIN_NAMES.length;
                if (gamepad1.left_bumper && !lastLb) selected = (selected + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
                if ((gamepad1.dpad_left && !lastLeft) || (gamepad1.dpad_right && !lastRight)) {
                    adjustGain(selected, gamepad1.dpad_right ? 1 : -1);
                }
                lastX = gamepad1.x; lastY = gamepad1.y; lastA = gamepad1.a;
                lastLb = gamepad1.left_bumper; lastRb = gamepad1.right_bumper;
                lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
                lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;

                setpoint = atHigh ? highRpm : LOW_RPM;

                // ---- step metrics ----
                double seconds = (System.nanoTime() - stepNanos) / 1e9;
                if (running && !Double.isNaN(actual)) {
                    overshoot = Math.max(overshoot, stepDir * (actual - setpoint));
                    if (Double.isNaN(settleSeconds) && flywheel.atSpeed()) settleSeconds = seconds;
                }

                // ---- telemetry (numeric values graph on Dashboard) ----
                telemetry.addData("target", running ? setpoint : 0);
                telemetry.addData("actual", Double.isNaN(actual) ? 0 : actual);
                telemetry.addData("error", running && !Double.isNaN(actual) ? setpoint - actual : 0);
                telemetry.addLine();
                telemetry.addData("state", running ? (atHigh ? "RUNNING at HIGH" : "RUNNING at LOW") : "stopped");
                telemetry.addData("HIGH / LOW rpm", "%.0f / %.0f", highRpm, LOW_RPM);
                telemetry.addData("encoders", flywheel.hasEncoder() ? "detected (PID on)" : running ? "NONE detected (feedforward only)" : "detected once spinning");
                telemetry.addData("direction", flywheel.isFlipped() ? "FLIPPED" : "normal");
                telemetry.addLine();
                for (int i = 0; i < GAIN_NAMES.length; i++) {
                    telemetry.addData((i == selected ? "> " : "  ") + GAIN_NAMES[i], gain(i));
                }
                telemetry.addLine();
                telemetry.addData("settle time (s)", Double.isNaN(settleSeconds) ? "not yet" : String.format("%.2f", settleSeconds));
                telemetry.addData("overshoot (rpm)", "%.0f (%.1f%%)", Math.max(0, overshoot),
                        setpoint > 0 ? 100 * Math.max(0, overshoot) / setpoint : 0);
                telemetry.addData("hint", running ? hint(seconds, setpoint, actual) : "press X");
                telemetry.addLine();
                telemetry.addData("paste into FlywheelSubsystem",
                        "kV = %.3f; kP = %.5f; kI = %.5f;",
                        FlywheelSubsystem.kV, FlywheelSubsystem.kP, FlywheelSubsystem.kI);
                telemetry.update();
            }
        } finally {
            flywheel.stop();
            flywheel.setRpm(originalRpm); // tuning shouldn't change the remembered shoot RPM
        }
    }

    private void startStep(double fromRpm, double toRpm) {
        stepNanos = System.nanoTime();
        stepDir = toRpm >= fromRpm ? 1 : -1;
        overshoot = 0;
        settleSeconds = Double.NaN;
    }

    private static double gain(int i) {
        switch (i) {
            case 0: return FlywheelSubsystem.kV;
            case 1: return FlywheelSubsystem.kP;
            default: return FlywheelSubsystem.kI;
        }
    }

    private static void adjustGain(int i, int sign) {
        double v = Math.max(0, gain(i) + sign * GAIN_STEPS[i]);
        switch (i) {
            case 0: FlywheelSubsystem.kV = v; break;
            case 1: FlywheelSubsystem.kP = v; break;
            default: FlywheelSubsystem.kI = v; break;
        }
    }

    /** One-line suggestion, only once the wheel has had a few seconds to settle. */
    private String hint(double seconds, double setpoint, double actual) {
        if (Double.isNaN(actual)) return "no encoder reading";
        if (seconds < 3) return "wait for it to settle...";
        double err = setpoint - actual;
        if (Math.abs(err) > FlywheelSubsystem.AT_SPEED_TOL * setpoint) {
            return err > 0 ? "stuck BELOW target: raise kV (then kI)" : "stuck ABOVE target: lower kV";
        }
        if (overshoot > 0.10 * setpoint) return "overshoot: lower kI or kP";
        if (seconds > 3 && settleSeconds > 2) return "slow: raise kP";
        return "looks good";
    }
}
