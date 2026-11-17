package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    private final Servo servoR, servoL;
    //private final AnalogInput turretEncoder;
    private Telemetry telemetry;
    private static final double GEAR_RATIO = 1.5;
    private static final double MAX_VOLTAGE = 3.3;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");
        //turretEncoder = hardwareMap.get(AnalogInput.class, "servoEncoder");
        this.telemetry = telemetry;
    }

    public void setTargetPosition(double position) {
        servoR.setPosition(position);
        servoL.setPosition(position);
    }
/*
    public double getServoAngleDegrees() {
        double normalized = Math.min(1.0, Math.max(0.0, turretEncoder.getVoltage() / MAX_VOLTAGE));
        return normalized * 360.0;
    }
    public double getTurretAngleDegrees() {
        return getServoAngleDegrees() * GEAR_RATIO;
    }

 */
}
