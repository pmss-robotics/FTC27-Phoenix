package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/** Intake driven by two motors. Rename the config names / flip directions to match your robot. */
public class DualIntakeSubsystem extends SubsystemBase {
    private static final double INTAKE_POWER = 1.0;

    private final DcMotorEx motorA, motorB;

    public DualIntakeSubsystem(HardwareMap hw) {
        motorA = hw.get(DcMotorEx.class, "intakeA");
        motorB = hw.get(DcMotorEx.class, "intakeB");
        // Flip one if the motors fight each other
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);
        motorB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void on() {
        motorA.setPower(INTAKE_POWER);
        motorB.setPower(INTAKE_POWER);
    }

    public void reverse() {
        motorA.setPower(-INTAKE_POWER);
        motorB.setPower(-INTAKE_POWER);
    }

    public void off() {
        motorA.setPower(0);
        motorB.setPower(0);
    }
}
