package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/** Pedro-driven robot-centric teleop + intake. Right trigger = in, left trigger = out. */
@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();

            if (gamepad1.right_trigger > 0.1) intake.on();
            else if (gamepad1.left_trigger > 0.1) intake.reverse();
            else intake.off();

            Pose p = follower.pose();
            telemetry.addData("x", p.x());
            telemetry.addData("y", p.y());
            telemetry.addData("heading (deg)", Math.toDegrees(p.heading()));
            telemetry.update();
        }
        follower.stop();
        intake.off();
    }
}
