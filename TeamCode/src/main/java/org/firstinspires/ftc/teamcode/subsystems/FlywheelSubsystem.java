package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Two flywheels run at the same RPM. The target RPM is remembered between opmode runs
 * (static field) and between app restarts (settings file). First ever run uses DEFAULT_RPM.
 */
public class FlywheelSubsystem extends SubsystemBase {
    public static final double DEFAULT_RPM = 3000;
    private static final double TICKS_PER_REV = 28; // goBILDA 5000-series bare motor encoder
    private static final double MAX_RPM = 6000;
    private static final String SAVE_FILE = "flywheelRpm.txt";

    // Both same direction. If both spin backwards, flip both. If one is backwards, flip just that one.
    private static final DcMotorSimple.Direction DIR_L = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction DIR_R = DcMotorSimple.Direction.FORWARD;

    private static double savedRpm = Double.NaN; // survives opmode restarts

    // Only one flywheel has an encoder. That one runs closed-loop velocity (the target RPM);
    // the other one just gets open-loop power estimated from the target RPM.
    private static final boolean ENCODER_ON_LEFT = true;  // set false if flywheelR has the encoder
    private static final double POWER_TRIM = 1.0;         // raise/lower if the no-encoder flywheel is slower/faster

    private final DcMotorEx flywheelL, flywheelR;
    private final DcMotorEx encoderMotor, powerMotor;
    private final File file = AppUtil.getInstance().getSettingsFile(SAVE_FILE);
    private double targetRpm;
    private boolean running = false;

    public FlywheelSubsystem(HardwareMap hw) {
        flywheelL = hw.get(DcMotorEx.class, "flywheelL");
        flywheelR = hw.get(DcMotorEx.class, "flywheelR");
        flywheelL.setDirection(DIR_L);
        flywheelR.setDirection(DIR_R);
        encoderMotor = ENCODER_ON_LEFT ? flywheelL : flywheelR;
        powerMotor = ENCODER_ON_LEFT ? flywheelR : flywheelL;
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        powerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        targetRpm = load();
    }

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

    /** Change the remembered target by delta RPM (clamped 0..MAX_RPM). */
    public void adjustRpm(double delta) {
        targetRpm = Math.max(0, Math.min(MAX_RPM, targetRpm + delta));
        save();
        apply();
    }

    /** Set the target RPM (clamped 0..MAX_RPM) and remember it. */
    public void setRpm(double rpm) {
        targetRpm = Math.max(0, Math.min(MAX_RPM, rpm));
        save();
        apply();
    }

    /** Spin up / stop. Only touches the motors when the state changes. */
    public void setRunning(boolean on) {
        if (on == running) return;
        running = on;
        apply();
    }

    public void toggle() {
        running = !running;
        apply();
    }

    public void stop() {
        running = false;
        apply();
    }

    private void apply() {
        if (running) {
            encoderMotor.setVelocity(targetRpm * TICKS_PER_REV / 60.0);
            powerMotor.setPower(Math.min(1.0, targetRpm / MAX_RPM * POWER_TRIM));
        } else {
            encoderMotor.setPower(0);
            powerMotor.setPower(0);
        }
    }

    public boolean isRunning() { return running; }
    public double getTargetRpm() { return targetRpm; }
    /** Measured RPM from the flywheel that has the encoder. */
    public double getActualRpm() { return encoderMotor.getVelocity() * 60.0 / TICKS_PER_REV; }
}
