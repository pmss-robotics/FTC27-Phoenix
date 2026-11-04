package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/** Template auto: drive to a pickup spot while intaking, then drive back. Edit poses/paths. */
@Autonomous(name = "TemplateAuto", group = "Auto")
public class TemplateAuto extends LinearOpMode {
    private static final Pose START = new Pose(0, 0, 0);
    private static final Pose PICKUP = new Pose(24, 0, 0);

    private Follower follower;

    private void runPath(Path path) {
        follower.follow(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("pose", follower.pose());
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        Path toPickup = line(START, PICKUP).constant(0);
        Path back = line(PICKUP, START).constant(0);

        waitForStart();
        if (isStopRequested()) return;

        intake.on();
        runPath(toPickup);
        intake.off();
        runPath(back);

        follower.stop();
    }
}
