package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/** Gate on one goBILDA servo (hardware name "gate"). Positions are final, tuned with GateTest. */
public class GateSubsystem extends SubsystemBase {
    public static final double BLOCKED_POS = 0.400; // was 0.420, pushed a bit further closed
    public static final double OPEN_POS = 0.520;

    private final Servo gate;

    public GateSubsystem(HardwareMap hw) {
        gate = hw.get(Servo.class, "gate");
        // Doesn't command the servo on init, so the gate doesn't jump until you tell it to.
    }

    public void block() {
        gate.setPosition(BLOCKED_POS);
    }

    public void open() {
        gate.setPosition(OPEN_POS);
    }

    public double getPosition() {
        return gate.getPosition();
    }
}
