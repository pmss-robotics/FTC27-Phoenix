package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

/**
 * Gate servo test. Gamepad1:
 *   A = go to BLOCKED_POS, B = go to OPEN_POS
 *   dpad up/down = nudge position +/-0.01 (hold bumpers for +/-0.05)
 *   Watch the position in telemetry, then copy the values you like into GateSubsystem.
 */
@TeleOp(name = "GateTest", group = "Test")
public class GateTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        boolean lastA = false, lastB = false, lastUp = false, lastDown = false;

        telemetry.addLine("Gate won't move until you press a button");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double step = (gamepad1.left_bumper || gamepad1.right_bumper) ? 0.05 : 0.01;

            if (gamepad1.a && !lastA) gate.block();
            if (gamepad1.b && !lastB) gate.open();
            if (gamepad1.dpad_up && !lastUp) gate.nudge(step);
            if (gamepad1.dpad_down && !lastDown) gate.nudge(-step);

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addData("gate position", "%.3f", gate.getPosition());
            telemetry.addData("BLOCKED_POS", GateSubsystem.BLOCKED_POS);
            telemetry.addData("OPEN_POS", GateSubsystem.OPEN_POS);
            telemetry.update();
        }
    }
}
