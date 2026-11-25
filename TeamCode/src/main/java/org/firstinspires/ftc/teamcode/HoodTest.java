package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

/**
 * Hood servo teleop. Gamepad1:
 *   A = hood to max position (servo 0), B = hood to lowest (servo about 0.19)
 *   dpad up/down = nudge servo +/-0.01 (hold a bumper for +/-0.05, up = lower hood)
 *   Position is clamped to the hood's range in HoodSubsystem.
 */
@TeleOp(name = "Hood", group = "Test")
public class HoodTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        HoodSubsystem hood = new HoodSubsystem(hardwareMap);
        boolean lastA = false, lastB = false, lastUp = false, lastDown = false;

        telemetry.addLine("Hood won't move until you press a button");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double step = (gamepad1.left_bumper || gamepad1.right_bumper) ? 0.05 : 0.01;

            if (gamepad1.a && !lastA) hood.max();
            if (gamepad1.b && !lastB) hood.low();
            if (gamepad1.dpad_up && !lastUp) hood.nudge(step);
            if (gamepad1.dpad_down && !lastDown) hood.nudge(-step);

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addData("hood servo pos", "%.3f", hood.getPosition());
            telemetry.addData("hood angle (deg from max)", "%.1f", hood.getAngle());
            telemetry.addData("LOW_POS limit", "%.3f", HoodSubsystem.LOW_POS);
            telemetry.update();
        }
    }
}
