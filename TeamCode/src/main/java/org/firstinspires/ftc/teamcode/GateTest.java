package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

/**
 * Standalone gate servo tuner (hardware name "gate"). Gamepad1:
 *   dpad left/right = pick which value you're editing: BLOCKED or OPEN
 *   dpad up/down    = change the selected value +/-0.01 (hold a bumper for +/-0.05)
 *                     the gate moves to the selected value live so you can see it
 *   A = go to BLOCKED value, B = go to OPEN value
 * Telemetry shows both values. Tell Claude the final BLOCKED and OPEN numbers.
 */
@TeleOp(name = "GateTest", group = "Test")
public class GateTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo gate = hardwareMap.get(Servo.class, "gate");

        // Final tuned values live in GateSubsystem (BLOCKED_POS / OPEN_POS).
        double blocked = GateSubsystem.BLOCKED_POS;
        double open = GateSubsystem.OPEN_POS;
        boolean editingBlocked = true;
        boolean lastA = false, lastB = false;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        telemetry.addLine("Gate won't move until you press a button");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double step = (gamepad1.left_bumper || gamepad1.right_bumper) ? 0.05 : 0.01;

            if (gamepad1.dpad_left && !lastLeft) {
                editingBlocked = true;
                gate.setPosition(blocked);
            }
            if (gamepad1.dpad_right && !lastRight) {
                editingBlocked = false;
                gate.setPosition(open);
            }

            double delta = 0;
            if (gamepad1.dpad_up && !lastUp) delta = step;
            if (gamepad1.dpad_down && !lastDown) delta = -step;
            if (delta != 0) {
                if (editingBlocked) {
                    blocked = clamp(blocked + delta);
                    gate.setPosition(blocked);
                } else {
                    open = clamp(open + delta);
                    gate.setPosition(open);
                }
            }

            if (gamepad1.a && !lastA) gate.setPosition(blocked);
            if (gamepad1.b && !lastB) gate.setPosition(open);

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            telemetry.addData("editing", editingBlocked ? "BLOCKED" : "OPEN");
            telemetry.addData("BLOCKED value", "%.3f", blocked);
            telemetry.addData("OPEN value", "%.3f", open);
            telemetry.addData("gate servo pos", "%.3f", gate.getPosition());
            telemetry.update();
        }
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
