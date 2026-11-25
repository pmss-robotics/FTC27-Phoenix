package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Two flywheels driven with the SAME power (same direction). Speed control:
 *   power = feedforward(rpm, battery voltage) + P + I on measured RPM.
 * Measured RPM comes from every flywheel whose encoder is detected (average if both, the one if one).
 * With no encoder detected, it runs open-loop on feedforward only. Call update() once per loop.
 *
 * Tune live from FTC Dashboard (fields below are public static), then paste the values here.
 * The shoot RPM is remembered between opmode runs (static) and app restarts (settings file).
 */
@Config
public class FlywheelSubsystem extends SubsystemBase {
    // ---- Hardware facts: goBILDA 5203 1:1 (6000 RPM free speed, 28 ticks/rev), 72mm Rhino wheel ----
    public static double MOTOR_FREE_RPM = 6000;
    public static double TICKS_PER_REV = 28;
    public static double NOMINAL_VOLTS = 12.0;
    public static double MAX_RPM = 6000;

    // Encoders are auto-detected: a flywheel counts once its encoder position moves off 0 (so a
    // flywheel with no encoder plugged in is ignored). Set one of these false to force-ignore it.
    public static boolean ENCODER_L = true;
    public static boolean ENCODER_R = true;

    // ---- Control gains ----
    // kV scales the feedforward: 1.0 = power exactly rpm / free speed (at 12 V).
    // Wheel/belt load makes the real value a bit above 1, so start slightly high and tune.
    public static double kV = 1.20645;
    public static double kP = 0.00075;   // power per RPM of error
    public static double kI = 0.0001;   // power per RPM-second of error
    public static double I_MAX_POWER = 0.20;  // cap on the integral's contribution
    public static double RPM_FILTER = 0.35;   // 0..1, higher = trusts new reading more
    public static double AT_SPEED_TOL = 0.03; // fraction of target considered "at speed"

    public static final double DEFAULT_RPM = 3000;
    private static final String SAVE_FILE = "flywheelRpm.txt";

    // L is this direction, R is the opposite. If they spin the wrong way, swap both with flipDirection().
    private static final DcMotorSimple.Direction DEFAULT_DIR = DcMotorSimple.Direction.FORWARD;

    private static double savedRpm = Double.NaN; // survives opmode restarts
    private static boolean flipped = false;      // survives opmode restarts

    private final DcMotorEx flywheelL, flywheelR;
    private final VoltageSensor battery;
    private final File file = AppUtil.getInstance().getSettingsFile(SAVE_FILE);

    private double targetRpm;      // remembered shoot RPM
    private double commandedRpm;   // what the flywheels are doing right now (negative = reverse)
    private double measuredRpm = Double.NaN;
    private boolean seenL = false, seenR = false; // encoder has been seen moving
    private double integral = 0;
    private long lastNanos = 0;
    private double volts = NOMINAL_VOLTS;
    private long lastVoltNanos = 0;

    public FlywheelSubsystem(HardwareMap hw) {
        flywheelL = hw.get(DcMotorEx.class, "flywheelL");
        flywheelR = hw.get(DcMotorEx.class, "flywheelR");
        battery = hw.voltageSensor.iterator().next();
        applyDirection();
        for (DcMotorEx m : new DcMotorEx[]{flywheelL, flywheelR}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // We close the loop ourselves, so power mode. Encoder velocity still reads in this mode.
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        targetRpm = load();
    }

    // ---------- direction ----------
    private void applyDirection() {
        DcMotorSimple.Direction d = flipped
                ? (DEFAULT_DIR == DcMotorSimple.Direction.REVERSE ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE)
                : DEFAULT_DIR;
        DcMotorSimple.Direction opposite = d == DcMotorSimple.Direction.REVERSE
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        flywheelL.setDirection(d);
        flywheelR.setDirection(opposite); // motors face each other, so R spins the opposite way
    }

    /** Flip the spin direction of both flywheels (use if they spin backwards). Remembered between runs. */
    public void flipDirection() {
        flipped = !flipped;
        applyDirection();
        integral = 0;
        measuredRpm = Double.NaN;
    }

    public boolean isFlipped() { return flipped; }

    // ---------- remembered RPM ----------
    private double load() {
        if (!Double.isNaN(savedRpm)) return savedRpm;
        try {
            if (file.exists()) {
                savedRpm = Double.parseDouble(ReadWriteFile.readFile(file).trim());
                return savedRpm;
            }
        } catch (Exception ignored) { }
        return DEFAULT_RPM;
    }

    private void save() {
        savedRpm = targetRpm;
        try {
            ReadWriteFile.writeFile(file, String.valueOf(targetRpm));
        } catch (Exception ignored) { }
    }

    // ---------- commands ----------
    public void adjustRpm(double delta) {
        setRpm(targetRpm + delta);
    }

    /** Set the shoot RPM (clamped 0..MAX_RPM) and remember it. Updates the motors if spinning forward. */
    public void setRpm(double rpm) {
        targetRpm = Math.max(0, Math.min(MAX_RPM, rpm));
        save();
        if (commandedRpm > 0) command(targetRpm);
    }

    /** Spin forward at the shoot RPM, or stop. */
    public void setRunning(boolean on) {
        double desired = on ? targetRpm : 0;
        if (desired != commandedRpm) command(desired);
    }

    /** Spin backwards at the given RPM (positive number). */
    public void reverse(double rpm) {
        double desired = -Math.abs(rpm);
        if (desired != commandedRpm) command(desired);
    }

    public void toggle() {
        setRunning(commandedRpm <= 0);
    }

    public void stop() {
        command(0);
    }

    private void command(double rpm) {
        commandedRpm = rpm;
        integral = 0;
        if (rpm == 0) setBothPower(0);
        else setBothPower(feedforward(rpm)); // immediate output; update() refines it
    }

    private void setBothPower(double p) {
        p = Math.max(-1.0, Math.min(1.0, p));
        flywheelL.setPower(p);
        flywheelR.setPower(p);
    }

    // ---------- control ----------
    private double feedforward(double rpm) {
        return rpm / MOTOR_FREE_RPM * kV * (NOMINAL_VOLTS / volts);
    }

    private void readVoltage(long now) {
        if (now - lastVoltNanos > 250_000_000L) { // refresh 4x/sec, the read is slow
            double v = battery.getVoltage();
            if (v > 6) volts = v;
            lastVoltNanos = now;
        }
    }

    /** Average RPM over flywheels with a working encoder (1 or 2); NaN if none detected. */
    private double readRpm() {
        // Only checks encoders not yet seen moving, so there's no extra reading once detected
        if (ENCODER_L && !seenL && flywheelL.getCurrentPosition() != 0) seenL = true;
        if (ENCODER_R && !seenR && flywheelR.getCurrentPosition() != 0) seenR = true;
        double sum = 0;
        int n = 0;
        if (ENCODER_L && seenL) { sum += flywheelL.getVelocity() * 60.0 / TICKS_PER_REV; n++; }
        if (ENCODER_R && seenR) { sum += flywheelR.getVelocity() * 60.0 / TICKS_PER_REV; n++; }
        return n == 0 ? Double.NaN : sum / n;
    }

    /** Call once every loop. */
    public void update() {
        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0.02 : (now - lastNanos) / 1e9;
        lastNanos = now;
        dt = Math.min(dt, 0.1);
        readVoltage(now);

        double raw = readRpm();
        if (!Double.isNaN(raw)) {
            measuredRpm = Double.isNaN(measuredRpm) ? raw
                    : measuredRpm + RPM_FILTER * (raw - measuredRpm);
        } else {
            measuredRpm = Double.NaN;
        }

        if (commandedRpm == 0) return;

        double power = feedforward(commandedRpm);
        if (!Double.isNaN(measuredRpm)) {
            double error = commandedRpm - measuredRpm;
            // Integrate only while not saturated, and cap how much the integral can contribute
            double maxI = kI > 0 ? I_MAX_POWER / kI : 0;
            if (Math.abs(power) < 1.0) {
                integral = Math.max(-maxI, Math.min(maxI, integral + error * dt));
            }
            power += kP * error + kI * integral;
        }
        setBothPower(power);
    }

    // ---------- status ----------
    public boolean isRunning() { return commandedRpm > 0; }
    public double getTargetRpm() { return targetRpm; }
    /** True once at least one flywheel encoder has been detected (happens right after the wheels first spin). */
    public boolean hasEncoder() { return (ENCODER_L && seenL) || (ENCODER_R && seenR); }

    /** Measured RPM (average of encoder flywheels), NaN if no encoders. Negative when reversing. */
    public double getActualRpm() { return measuredRpm; }

    public boolean atSpeed() {
        return commandedRpm != 0 && !Double.isNaN(measuredRpm)
                && Math.abs(commandedRpm - measuredRpm) <= AT_SPEED_TOL * Math.abs(commandedRpm);
    }
}
