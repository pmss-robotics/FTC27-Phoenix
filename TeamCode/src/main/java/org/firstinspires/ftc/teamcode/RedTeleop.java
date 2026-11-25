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

/**
 * Red teleop. Field-centric drive, odometry-aimed turret, intake, 2 flywheels.
 *
 * Gamepad1:
 *   left stick = drive, right stick X = turn (field-centric)
 *   right bumper (hold) = intake
 *   X = flywheels on/off, dpad up/down = target RPM +/-100 (remembered between runs)
 *   Y = unlock/lock the turret (locked = centered at 0.5, unlocked = auto-aims at the HIVE)
 *   START = reset pose to the start pose
 *
 * Field frame (inches): origin at a field corner, field is 144x144, red half is x < 72.
 */
@TeleOp(name = "RedTeleop", group = "Main")
public class RedTeleop extends LinearOpMode {
    // ---- Start pose: robot against the bottom wall (y = 0), back to the wall, facing +y ----
    private static final double ROBOT_WIDTH_IN = 300 / 25.4;   // 11.81
    private static final double ROBOT_LENGTH_IN = 400 / 25.4;  // 15.75
    private static final double START_X = 36;                       // inches along the wall
    private static final double START_Y = ROBOT_LENGTH_IN / 2;      // back flush with the wall
    private static final double START_HEADING = Math.PI / 2;       // facing +y (radians)

    // ---- Aim target: HIVE (center of the field). Adjust to the real CELL you shoot at ----
    private static final double TARGET_X = 72;
    private static final double TARGET_Y = 72;

    // ---- Turret mapping (tune these) ----
    private static final double SERVO_RANGE_DEG = 300; // servo travel for position 0 -> 1
    private static final double TURRET_RATIO = 1.5;     // turret degrees per servo degree
    private static final double TURRET_SIGN = 1;        // flip to -1 if the turret aims the wrong way
    private static final double LOCKED_POSITION = 0.5;  // turret centered

    private static final double RPM_STEP = 100;

    @Override
    public void runOpMode() {
        Pose startPose = new Pose(START_X, START_Y, START_HEADING);
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        DualIntakeSubsystem intake = new DualIntakeSubsystem(hardwareMap);
        FlywheelSubsystem flywheel = new FlywheelSubsystem(hardwareMap);
        AxonSubsystem axons = new AxonSubsystem(hardwareMap); // starts at 0.5 (locked, centered)

        boolean turretUnlocked = false;
        boolean lastX = false, lastY = false, lastUp = false, lastDown = false;

        telemetry.addData("start pose", "x=%.1f y=%.1f heading=%.0f deg",
                START_X, START_Y, Math.toDegrees(START_HEADING));
        telemetry.addData("flywheel target RPM", flywheel.getTargetRpm());
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            Pose pose = follower.pose();

            // Drive
            follower.manual(ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    pose.heading()));
            follower.update();
            flywheel.update();

            // Intake
            if (gamepad1.right_bumper) intake.on();
            else intake.off();

            // Flywheels
            if (gamepad1.x && !lastX) flywheel.toggle();
            if (gamepad1.dpad_up && !lastUp) flywheel.adjustRpm(RPM_STEP);
            if (gamepad1.dpad_down && !lastDown) flywheel.adjustRpm(-RPM_STEP);

            // Turret lock / unlock
            if (gamepad1.y && !lastY) turretUnlocked = !turretUnlocked;

            double aimedPos = aimPosition(pose);
            axons.setPosition(turretUnlocked ? aimedPos : LOCKED_POSITION);

            if (gamepad1.start) follower.setPose(startPose);

            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addData("pose", "x=%.1f y=%.1f h=%.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("turret", turretUnlocked ? "UNLOCKED (aiming)" : "LOCKED (center)");
            telemetry.addData("turret servo pos", axons.getPosition());
            telemetry.addData("flywheel", flywheel.isRunning() ? "ON" : "off");
            telemetry.addData("target RPM", flywheel.getTargetRpm());
            telemetry.addData("actual RPM", flywheel.hasEncoder() ? String.format("%.0f%s", flywheel.getActualRpm(), flywheel.atSpeed() ? " (at speed)" : "") : "no encoder");
            telemetry.update();
        }
        follower.stop();
        intake.off();
        flywheel.stop();
    }

    /** Servo position (0..1) that points the turret at the target from the current pose. */
    private double aimPosition(Pose pose) {
        double toTarget = Math.atan2(TARGET_Y - pose.y(), TARGET_X - pose.x());
        double rel = toTarget - pose.heading();                    // relative to robot forward
        rel = Math.atan2(Math.sin(rel), Math.cos(rel));            // wrap to -pi..pi
        double servoDeg = Math.toDegrees(rel) / TURRET_RATIO;
        double pos = LOCKED_POSITION + TURRET_SIGN * servoDeg / SERVO_RANGE_DEG;
        return Math.max(0, Math.min(1, pos));
    }
}
