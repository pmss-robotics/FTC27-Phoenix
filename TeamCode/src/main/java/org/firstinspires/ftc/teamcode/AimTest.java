package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AxonSubsystem;

/**
 * Turret auto-aim test using odometry. Gamepad1: left stick drive, right stick X turn (field-centric),
 * dpad left/right = trim the aim (degrees), nothing else.
 *
 * FIELD FRAME (my own, inches, 144 x 144): origin = back-wall corner on the robot's right,
 *   +x = toward the audience (robot heading 0 faces +x), +y = robot's LEFT = BLUE side.
 * START: back wall, flush, centered behind the blue mouth, facing +x.
 *
 * AIM: always at the center of the blue mouth (fixed point).
 *
 * TURRET: servo 0.5 = turret faces BACK, increasing servo = turret turns CLOCKWISE, and the 25:50 x 20:60
 * gearing makes 1 servo degree = 1.5 turret degrees (Axon range 350 deg, so 525 turret deg total).
 */
@TeleOp(name = "AimTest", group = "Test")
public class AimTest extends LinearOpMode {
    // ---- Robot ----
    private static final double ROBOT_LENGTH = 16.5;
    private static final double TURRET_BACK_OFFSET = 2.0; // turret pivot is this far BEHIND robot center

    // ---- Field / hive. From the manual: hive frame 49.46 wide x 38.95 deep in the middle of the field,
    // ---- cells 18.8 apart, mouth opening 20 wide. HIVE_CENTER_Y and the tilt are my ESTIMATES: measure and fix.
    private static final double HIVE_CENTER_X = 72.0;
    private static final double CELL_SEPARATION = 18.8;
    private static final double HIVE_CENTER_Y = 84.4;     // blue hive centerline (estimate: 72 + 12.4)
    private static final double TARGET_X = HIVE_CENTER_X - CELL_SEPARATION / 2.0; // mouth nearest the robot

    // ---- Turret ----
    private static final double AXON_RANGE_DEG = 350.0;           // Axon MK2 max rotation (goBILDA spec)
    private static final double GEAR = 1.5;                       // turret deg per servo deg
    private static final double TURRET_RANGE_DEG = AXON_RANGE_DEG * GEAR; // 525
    private static final double TRIM_STEP_DEG = 1.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        // Back wall flush, directly behind the blue mouth, facing the audience.
        follower.setPose(new Pose(ROBOT_LENGTH / 2.0, HIVE_CENTER_Y, 0));

        AxonSubsystem axons = new AxonSubsystem(hardwareMap);
        axons.useFullPwmRange(); // 500-2500 us so 0..1 really is 350 deg
        double trimDeg = 0;
        boolean lastLeft = false, lastRight = false;

        waitForStart();
        while (opModeIsActive()) {
            follower.manual(ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()));
            follower.update();

            if (gamepad1.dpad_left && !lastLeft) trimDeg -= TRIM_STEP_DEG;
            if (gamepad1.dpad_right && !lastRight) trimDeg += TRIM_STEP_DEG;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            Pose pose = follower.pose();
            double h = pose.heading();

            // Turret pivot in field coords (2 in behind robot center).
            double turretX = pose.x() - TURRET_BACK_OFFSET * Math.cos(h);
            double turretY = pose.y() - TURRET_BACK_OFFSET * Math.sin(h);

            double aimX = TARGET_X;
            double aimY = HIVE_CENTER_Y;

            // Angle to the aim point, then relative to the robot's front (CCW positive).
            double fieldAngle = Math.atan2(aimY - turretY, aimX - turretX);
            double relDeg = normalizeDeg(Math.toDegrees(fieldAngle - h) + trimDeg);

            // Turret angle from front (CCW+) = 180 - (servo - 0.5) * TURRET_RANGE_DEG. Solve for servo, picking the wrap
            // (+/-360) that lands in [0,1] and is closest to where the turret already is.
            double bestPos = Double.NaN;
            for (int k = -1; k <= 1; k++) {
                double pos = 0.5 + (180.0 - (relDeg + 360.0 * k)) / TURRET_RANGE_DEG;
                if (pos < 0 || pos > 1) continue;
                if (Double.isNaN(bestPos) || Math.abs(pos - axons.getPosition()) < Math.abs(bestPos - axons.getPosition())) {
                    bestPos = pos;
                }
            }
            if (!Double.isNaN(bestPos)) axons.setPosition(bestPos);

            telemetry.addData("pose", "x=%.1f y=%.1f h=%.0f", pose.x(), pose.y(), Math.toDegrees(h));
            telemetry.addData("aim point", "x=%.1f y=%.1f", aimX, aimY);
            telemetry.addData("turret rel angle (deg)", "%.1f", relDeg);
            telemetry.addData("servo pos", "%.3f", axons.getPosition());
            telemetry.addData("trim (deg)", trimDeg);
            telemetry.update();
        }
        follower.stop();
    }

    /** Wrap to (-180, 180]. */
    private static double normalizeDeg(double d) {
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
    }
}
