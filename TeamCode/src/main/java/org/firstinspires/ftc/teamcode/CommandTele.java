package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name="use me")
public class CommandTele extends CommandOpMode {

    @Override
    public void initialize() {
        DriveSubsystem drive = new DriveSubsystem(Constants.create(hardwareMap), Pose.zero(), telemetry);
        drive.follower.setPose(Pose.zero());
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);


        new Trigger(()->gamepad1.left_trigger>0.1)
                .whenActive(new InstantCommand(intake::on));

        new Trigger(()->gamepad1.right_trigger>0.1)
                .whenActive(new InstantCommand(intake::reverse));

        schedule(drive.drive(
                ()-> gamepad1.left_stick_y,
                ()-> gamepad1.left_stick_x,
                ()-> gamepad1.right_stick_x
        ));


    }
}
