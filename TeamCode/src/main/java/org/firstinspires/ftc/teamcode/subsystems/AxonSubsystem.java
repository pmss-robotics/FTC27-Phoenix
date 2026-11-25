package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/** Two Axon servos driven together, same direction. Position control (servo mode), 0..1. */
public class AxonSubsystem extends SubsystemBase {
    private final Servo servoR, servoL;
    private final AnalogInput encoder; // Axon feedback wire, read from one servo
    private double position = 0.5;

    public AxonSubsystem(HardwareMap hw) {
        servoR = hw.get(Servo.class, "servoR");
        servoL = hw.get(Servo.class, "servoL");
        encoder = hw.tryGet(AnalogInput.class, "servoEncoder"); // null if not in config
        // Both same direction. If they fight, mount/phase is the issue, or reverse one here.
        servoR.setDirection(Servo.Direction.FORWARD);
        servoL.setDirection(Servo.Direction.FORWARD);
        setPosition(position);
    }

    /**
     * Widen both servos' pulse range to 500-2500 us so position 0..1 covers the full 350 deg
     * (the hub default 600-2400 us only gives about 315 deg). The center (0.5) stays the same.
     */
    public void useFullPwmRange() {
        for (Servo s : new Servo[]{servoR, servoL}) {
            if (s instanceof ServoImplEx) {
                ((ServoImplEx) s).setPwmRange(new PwmControl.PwmRange(500, 2500));
            }
        }
    }

    /** Move the target by delta (clamped 0..1). */
    public void nudge(double delta) {
        setPosition(position + delta);
    }

    public void setPosition(double p) {
        position = Math.max(0.0, Math.min(1.0, p));
        servoR.setPosition(position);
        servoL.setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    /** Actual servo output angle from the feedback wire, 0..360, or NaN if no encoder is configured. */
    public double getAngleDegrees() {
        if (encoder == null) return Double.NaN;
        return encoder.getVoltage() / encoder.getMaxVoltage() * 360.0;
    }
}
