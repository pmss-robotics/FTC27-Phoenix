package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//imports
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Flywheel implements Subsystem {
    private final DcMotorEx motor; //declare motor
    private final Telemetry telemetry; //declare telemetry
    private double targetRpm; //variable for targetRpm
    public static double kP =  340.0;
    public static double kI =   0.0;
    public static double kD =   0.0;
    public static double kF =  13.15;  // 32767 / (6000/60 * 28)

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM       = 6000.0;

    // Tracks last-applied gains so periodic() only writes when they change.
    private double lastKP, lastKI, lastKD, lastKF;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        //motor setup
        motor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocityRpm(double rpm){
        targetRpm = rpm;
        ensureEncoderMode();
        if (rpm == 0) {
            motor.setPower(0);
            return;
        }

        double tps = (rpm / 60.0) * TICKS_PER_REV;
        motor.setVelocity(tps);
    }

    private void ensureEncoderMode() {
        if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyGains();
        }
    }

    private void applyGains() {
        PIDFCoefficients c = new PIDFCoefficients(kP, kI, kD, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        lastKP = kP; lastKI = kI; lastKD = kD; lastKF = kF;
    }

    @Override
    public void periodic() {
        if (kP != lastKP || kI != lastKI || kD != lastKD || kF != lastKF) {
            applyGains();
        }
        telemetry.addData("Target RPM",   targetRpm);
        telemetry.addData("Actual RPM", getActualRpm());
    }

    public double getActualRpm()  { return motor.getVelocity() * 60.0 / TICKS_PER_REV; }
    public double getTargetRPM()  { return targetRpm; }
}
