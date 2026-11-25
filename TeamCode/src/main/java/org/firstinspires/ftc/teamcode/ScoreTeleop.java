package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AxonSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DualIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

/**
 * Intake / shoot cycle with a gate, field-centric, starts at (0, 0, 0). Turret fixed at the front (not aiming).
 *
 * Gamepad1:
 *   left stick = drive, right stick X = turn (field-centric)
 *   hold right bumper = intake on, gate BLOCKED (holds balls in), flywheels off
 *   release right bumper = short intake backoff, then flywheels spin up (gate still blocked)
 *   hold Y = gate OPEN and intake feeds the shot
 *   release Y = gate blocked, flywheels and intake off, ready for intake again
 *   A = hood to its highest position, B = hood to its lowest (starts at highest)
 *   left bumper = flip flywheel direction (if they spin backwards)
 *   dpad left/right = nudge turret (calibration)
 *   dpad up/down = shoot RPM +/-100 (starts at 3000 each run)
 */
@TeleOp(name = "ScoreTeleop", group = "Main")
public class ScoreTeleop extends LinearOpMode {
    private static final double START_RPM = 3000;
    private static final double RPM_STEP = 100;
    private static final long BACKOFF_MS = 25;          // how long the pre-shot intake backoff lasts
    // Turret faces the BACK at 0.5 (same as AimTest). Front is 180 turret-deg away. With the Axon's 350 deg
    // range (full PWM range) and 1:1.5 gearing, 1.0 of servo = 525 turret-deg, so front is
    // 0.5 -/+ 180/525 = 0.157 or 0.843. We use 0.157 (the low side). Trim with dpad if it's a hair off.
    private static final double TURRET_FRONT_POSITION = 0.5 - 180.0 / (350.0 * 1.5);
    private static final double TURRET_NUDGE = 0.005;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        DualIntakeSubsystem intake = new DualIntakeSubsystem(hardwareMap);
        FlywheelSubsystem flywheel = new FlywheelSubsystem(hardwareMap);
        AxonSubsystem axons = new AxonSubsystem(hardwareMap);
        axons.useFullPwmRange(); // 500-2500 us so 0..1 is the full 350 deg, same as AimTest
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        HoodSubsystem hood = new HoodSubsystem(hardwareMap);

        flywheel.setRpm(START_RPM);
        double turretPos = TURRET_FRONT_POSITION;
        axons.setPosition(turretPos);
        gate.block();
        hood.max();

        boolean spinning = false;   // flywheels held on forward after releasing intake
        long backoffUntil = 0;      // ms timestamp the backoff ends
        boolean lastRb = false, lastY = false, lastUp = false, lastDown = false;
        boolean lastLeft = false, lastRight = false, lastLb = false;
        boolean lastA = false, lastB = false;

        waitForStart();
        while (opModeIsActive()) {
            boolean rb = gamepad1.right_bumper;
            boolean y = gamepad1.y;
            long now = System.currentTimeMillis();

            // Drive
            follower.manual(ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()));
            follower.update();
            flywheel.update();

            // Cycle state
            if (rb && !lastRb) spinning = false;              // started intaking
            if (lastRb && !rb && !y) {                        // let go of intake: go shoot
                backoffUntil = now + BACKOFF_MS;
                spinning = true;
            }
            if (lastY && !y) spinning = false;                // let go of Y: everything off

            boolean backingOff = now < backoffUntil;
            if (y) {
                gate.open();
                intake.on();
                flywheel.setRunning(true);
            } else if (rb) {
                gate.block();
                intake.on();
                flywheel.setRunning(false);
            } else if (backingOff) {
                gate.block();
                intake.reverse();
                flywheel.setRunning(false);
            } else {
                gate.block();
                intake.off();
                flywheel.setRunning(spinning);
            }

            // Flip flywheel direction (if they spin backwards)
            if (gamepad1.left_bumper && !lastLb) flywheel.flipDirection();

            // Hood
            if (gamepad1.a && !lastA) hood.max();
            if (gamepad1.b && !lastB) hood.low();

            // RPM adjust
            if (gamepad1.dpad_up && !lastUp) flywheel.adjustRpm(RPM_STEP);
            if (gamepad1.dpad_down && !lastDown) flywheel.adjustRpm(-RPM_STEP);

            // Turret calibration: dpad left/right nudges it, then copy the value from telemetry
            if (gamepad1.dpad_left && !lastLeft) turretPos -= TURRET_NUDGE;
            if (gamepad1.dpad_right && !lastRight) turretPos += TURRET_NUDGE;
            turretPos = Math.max(0, Math.min(1, turretPos));
            axons.setPosition(turretPos);

            lastRb = rb;
            lastY = y;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastLb = gamepad1.left_bumper;
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            Pose p = follower.pose();
            telemetry.addData("pose", "x=%.1f y=%.1f h=%.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("turret pos (copy into TURRET_FRONT_POSITION)", "%.3f", turretPos);
            telemetry.addData("flywheel direction", flywheel.isFlipped() ? "FLIPPED" : "normal");
            telemetry.addData("hood servo pos", "%.3f", hood.getPosition());
            telemetry.addData("shoot RPM", flywheel.getTargetRpm());
            telemetry.addData("actual RPM", flywheel.hasEncoder() ? String.format("%.0f%s", flywheel.getActualRpm(), flywheel.atSpeed() ? " (at speed)" : "") : "no encoder");
            telemetry.update();
        }
        follower.stop();
        intake.off();
        flywheel.stop();
    }
}
