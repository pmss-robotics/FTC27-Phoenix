package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/** Plain (no Pedro) drive + intake. Right trigger = intake in, left trigger = out. */
@TeleOp(name = "MecanumTestOp", group = "Test")
public class MecanumTestOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.right_trigger > 0.1) intake.on();
            else if (gamepad1.left_trigger > 0.1) intake.reverse();
            else intake.off();

            telemetry.addLine("robot-centric mecanum + intake");
            telemetry.update();
        }
        drive.stop();
        intake.off();
    }
}
