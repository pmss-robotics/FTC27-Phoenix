package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DualIntakeSubsystem;

/** Template field-centric teleop. Right bumper = run both intake motors. Edit START_* to change start pose. */
@TeleOp(name = "TemplateTeleop", group = "Main")
public class TemplateTeleop extends LinearOpMode {
    // Starting pose (inches, radians). Change these to wherever the robot is placed.
    private static final double START_X = 0;
    private static final double START_Y = 0;
    private static final double START_HEADING = 0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, START_HEADING));
        DualIntakeSubsystem intake = new DualIntakeSubsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            // Stick input in field terms: forward = +x, left = +y
            double fieldFwd = -gamepad1.left_stick_y;
            double fieldLeft = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // Rotate into the robot frame using current heading
            double h = follower.pose().heading();
            double robotFwd = fieldFwd * Math.cos(h) + fieldLeft * Math.sin(h);
            double robotLeft = -fieldFwd * Math.sin(h) + fieldLeft * Math.cos(h);

            follower.manual(robotFwd, robotLeft, turn);
            follower.update();

            if (gamepad1.right_bumper) intake.on();
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
