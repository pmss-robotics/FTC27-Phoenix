package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "MecanumTestOp", group = "Test")
public class MecanumTestOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addLine("robot-centric mecanum");
            telemetry.update();
        }
        drive.stop();
    }
}
