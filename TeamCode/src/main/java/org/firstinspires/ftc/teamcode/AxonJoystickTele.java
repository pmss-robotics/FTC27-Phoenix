package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AxonSubsystem;

/** Left stick X moves both Axons together. Starts at 0.5. */
@TeleOp(name = "AxonJoystickTele", group = "Test")
public class AxonJoystickTele extends LinearOpMode {
    // Position change per loop at full stick. Lower = slower.
    private static final double SPEED = 0.05;
    private static final double DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        AxonSubsystem axons = new AxonSubsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double stick = gamepad1.left_stick_x;
            if (Math.abs(stick) > DEADBAND) axons.nudge(stick * SPEED);

            telemetry.addData("target position", axons.getPosition());
            telemetry.addData("actual angle (deg)", axons.getAngleDegrees());
            telemetry.update();
        }
    }
}
