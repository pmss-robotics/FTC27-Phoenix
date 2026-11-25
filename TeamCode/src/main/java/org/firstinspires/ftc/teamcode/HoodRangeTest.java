package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

/**
 * Hood direction + range finder (hardware name "hood"). Not clamped, so you can find the real limits.
 * Gamepad1:
 *   dpad left/right = pick which end you're editing: MAX (hood fully up) or LOW (hood fully down)
 *   dpad up/down    = change the selected value +/-0.01 (hold a bumper for +/-0.05)
 *                     the hood moves to the selected value live so you can see it
 *   A = go to the MAX value, B = go to the LOW value
 * Which way is "up" is whatever you see: watch the hood while you press dpad up/down.
 * Telemetry shows both values and the hood angle between them (using the 20:115 gear ratio).
 * Tell Claude the final MAX and LOW numbers (and which direction servo up/down moves the hood).
 * Go slowly near the ends so the hood doesn't slam into a hard stop.
 */
@TeleOp(name = "HoodRangeTest", group = "Test")
public class HoodRangeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo hood = hardwareMap.get(Servo.class, "hood");

        double max = HoodSubsystem.MAX_POS;
        double low = HoodSubsystem.LOW_POS;
        boolean editingMax = true;
        boolean lastA = false, lastB = false;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        telemetry.addLine("Hood won't move until you press a button");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double step = (gamepad1.left_bumper || gamepad1.right_bumper) ? 0.05 : 0.01;

            if (gamepad1.dpad_left && !lastLeft) {
                editingMax = true;
                hood.setPosition(max);
            }
            if (gamepad1.dpad_right && !lastRight) {
                editingMax = false;
                hood.setPosition(low);
            }

            double delta = 0;
            if (gamepad1.dpad_up && !lastUp) delta = step;
            if (gamepad1.dpad_down && !lastDown) delta = -step;
            if (delta != 0) {
                if (editingMax) {
                    max = clamp(max + delta);
                    hood.setPosition(max);
                } else {
                    low = clamp(low + delta);
                    hood.setPosition(low);
                }
            }

            if (gamepad1.a && !lastA) hood.setPosition(max);
            if (gamepad1.b && !lastB) hood.setPosition(low);

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            double rangeServoDeg = Math.abs(low - max) * HoodSubsystem.SERVO_RANGE_DEG;
            telemetry.addData("editing", editingMax ? "MAX" : "LOW");
            telemetry.addData("MAX value", "%.3f", max);
            telemetry.addData("LOW value", "%.3f", low);
            telemetry.addData("servo pos", "%.3f", hood.getPosition());
            telemetry.addData("range (servo deg)", "%.1f", rangeServoDeg);
            telemetry.addData("range (hood deg)", "%.1f", rangeServoDeg * HoodSubsystem.GEAR_RATIO);
            telemetry.update();
        }
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
