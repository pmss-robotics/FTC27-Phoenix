package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Hood on one goBILDA speed servo (servo mode, 300 deg range), hardware name "hood".
 * Measured with HoodRangeTest: servo 0.300 = hood at its highest, 0.412 = hood at its lowest
 * (higher servo value = lower hood).
 * Gear ratio servo:hood is 20:115, so the hood moves 20/115 as far as the servo.
 */
public class HoodSubsystem extends SubsystemBase {
    public static final double SERVO_RANGE_DEG = 300.0;
    public static final double GEAR_RATIO = 20.0 / 115.0; // hood degrees per servo degree

    public static final double MAX_POS = 0.300; // hood at its highest
    public static final double LOW_POS = 0.412; // hood at its lowest
    /** Total hood travel in degrees, from the measured servo range. */
    public static final double HOOD_TRAVEL_DEG = (LOW_POS - MAX_POS) * SERVO_RANGE_DEG * GEAR_RATIO;

    private final Servo hood;
    private double position = MAX_POS;

    public HoodSubsystem(HardwareMap hw) {
        hood = hw.get(Servo.class, "hood");
        // Doesn't command the servo on init, so the hood doesn't jump until you tell it to.
    }

    /** Move the hood to a servo position, clamped to MAX_POS..LOW_POS. */
    public void setPosition(double p) {
        position = Math.max(MAX_POS, Math.min(LOW_POS, p));
        hood.setPosition(position);
    }

    /** Hood angle in degrees down from max (0 = max position). */
    public void setAngle(double hoodDeg) {
        setPosition(MAX_POS + hoodDeg / GEAR_RATIO / SERVO_RANGE_DEG);
    }

    public double getAngle() {
        return (position - MAX_POS) * SERVO_RANGE_DEG * GEAR_RATIO;
    }

    public void nudge(double delta) {
        setPosition(position + delta);
    }

    public void max() {
        setPosition(MAX_POS);
    }

    public void low() {
        setPosition(LOW_POS);
    }

    public double getPosition() {
        return position;
    }
}
