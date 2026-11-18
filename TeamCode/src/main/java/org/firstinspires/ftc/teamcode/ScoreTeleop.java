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

/**
 * Intake / shoot cycle, field-centric, starts at (0, 0, 0). Turret always at 0.5.
 *
 * Gamepad1:
 *   left stick = drive, right stick X = turn (field-centric)
 *   hold right bumper = intake on, gate BLOCKED
 *   release right bumper = flywheels spin up automatically
 *   hold Y = gate OPEN + intake on (feeds shots)
 *   release Y = flywheels, intake off, gate blocked (ready for intake again)
 *   left bumper = swap the blocked and open gate positions
 *   dpad up/down = flywheel RPM +/-100 (starts at 3000 each run)
 */
@TeleOp(name = "ScoreTeleop", group = "Main")
public class ScoreTeleop extends LinearOpMode {
    private static final double START_RPM = 3000;
    private static final double RPM_STEP = 100;
    private static final double TURRET_POSITION = 0.5;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        DualIntakeSubsystem intake = new DualIntakeSubsystem(hardwareMap);
        FlywheelSubsystem flywheel = new FlywheelSubsystem(hardwareMap);
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        AxonSubsystem axons = new AxonSubsystem(hardwareMap);

        flywheel.setRpm(START_RPM);
        gate.block();
        axons.setPosition(TURRET_POSITION);

        boolean spinning = false; // flywheels held on after releasing intake
        boolean lastLb = false, lastRb = false, lastY = false, lastUp = false, lastDown = false;

        waitForStart();
        while (opModeIsActive()) {
            boolean rb = gamepad1.right_bumper;
            boolean y = gamepad1.y;

            // Drive
            follower.manual(ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()));
            follower.update();

            if (gamepad1.left_bumper && !lastLb) gate.toggleSwap();

            // Cycle state
            if (rb && !lastRb) spinning = false;          // started intaking: flywheels off
            if (lastRb && !rb && !y) spinning = true;     // let go of intake: spin up
            if (lastY && !y) spinning = false;            // let go of Y: everything off

            if (y) {
                gate.open();
                intake.on();
            } else if (rb) {
                gate.block();
                intake.on();
            } else {
                gate.block();
                intake.off();
            }
            flywheel.setRunning(spinning || y);

            // RPM adjust
            if (gamepad1.dpad_up && !lastUp) flywheel.adjustRpm(RPM_STEP);
            if (gamepad1.dpad_down && !lastDown) flywheel.adjustRpm(-RPM_STEP);

            axons.setPosition(TURRET_POSITION);

            lastLb = gamepad1.left_bumper;
            lastRb = rb;
            lastY = y;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            Pose p = follower.pose();
            telemetry.addData("pose", "x=%.1f y=%.1f h=%.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("gate", "%.2f%s", gate.getPosition(), gate.isSwapped() ? " (SWAPPED)" : "");
            telemetry.addData("flywheel", flywheel.isRunning() ? "ON" : "off");
            telemetry.addData("target RPM", flywheel.getTargetRpm());
            telemetry.addData("actual RPM", "%.0f", flywheel.getActualRpm());
            telemetry.update();
        }
        follower.stop();
        intake.off();
        flywheel.stop();
        gate.block();
    }
}
