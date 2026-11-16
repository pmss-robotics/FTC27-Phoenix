package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Flywheel velocity test. Gamepad1: X toggles flywheel on/off, dpad up/down = +/-100 RPM.
 * Edit MOTOR_NAME / TICKS_PER_REV / REVERSED below if needed.
 */
@TeleOp(name = "FlywheelTest", group = "Test")
public class FlywheelTest extends LinearOpMode {
    private static final String MOTOR_NAME = "flywheel";
    private static final double TICKS_PER_REV = 28; // goBILDA 5000-series bare motor encoder
    private static final boolean REVERSED = false;
    private static final double STEP_RPM = 100;

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        flywheel.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double targetRpm = 4000;
        boolean running = false;
        boolean lastX = false, lastUp = false, lastDown = false;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x && !lastX) running = !running;
            if (gamepad1.dpad_up && !lastUp) targetRpm += STEP_RPM;
            if (gamepad1.dpad_down && !lastDown) targetRpm = Math.max(0, targetRpm - STEP_RPM);
            lastX = gamepad1.x;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            if (running) flywheel.setVelocity(targetRpm * TICKS_PER_REV / 60.0);
            else flywheel.setPower(0);

            double actualRpm = flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
            telemetry.addData("running", running);
            telemetry.addData("target RPM", targetRpm);
            telemetry.addData("actual RPM", "%.0f", actualRpm);
            telemetry.addData("error", "%.0f", targetRpm - actualRpm);
            telemetry.update();
        }
        flywheel.setPower(0);
    }
}
