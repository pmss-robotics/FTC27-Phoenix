package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/** Gate on one goBILDA servo. Tune BLOCKED_POS / OPEN_POS with GateTest. */
public class GateSubsystem extends SubsystemBase {
    // Placeholders: find the real values with GateTest, then paste them here.
    public static final double BLOCKED_POS = 0.0;
    public static final double OPEN_POS = 0.5;

    private final Servo gate;
    private double position = BLOCKED_POS;
    private boolean swapped = false; // true = block() and open() use each other's positions

    public GateSubsystem(HardwareMap hw) {
        gate = hw.get(Servo.class, "gate");
        // Doesn't command the servo on init, so the gate doesn't jump until you tell it to.
    }

    public void block() {
        setPosition(swapped ? OPEN_POS : BLOCKED_POS);
    }

    public void open() {
        setPosition(swapped ? BLOCKED_POS : OPEN_POS);
    }

    /** Swap which position counts as blocked vs open (for when you mixed them up). */
    public void toggleSwap() {
        swapped = !swapped;
    }

    public boolean isSwapped() {
        return swapped;
    }

    /** Move the gate to any position (clamped 0..1). */
    public void setPosition(double p) {
        position = Math.max(0.0, Math.min(1.0, p));
        gate.setPosition(position);
    }

    public void nudge(double delta) {
        setPosition(position + delta);
    }

    public double getPosition() {
        return position;
    }
}
