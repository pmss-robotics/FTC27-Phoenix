package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;


@Config
public class DriveSubsystem extends SubsystemBase {
    public Follower follower;
    private Telemetry telemetry;
    public DriveSubsystem(Follower follower, Pose startPose, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        follower.setPose(startPose);
    }

    @Override
    public void periodic() {
        follower.update();

        Pose pose = getPose();
        telemetry.addData("x", pose.x());
        telemetry.addData("y",pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
    }

    public Command drive(DoubleSupplier ly, DoubleSupplier lx, DoubleSupplier rx) {
        return new RunCommand(()->{
            follower.manual(ManualDrive.fieldCentric(
                    -ly.getAsDouble(),
                    -lx.getAsDouble(),
                    -rx.getAsDouble(),
                    follower.pose().heading()
            ));
            follower.update();
        }
    );

    }
    public Pose getPose() {
        return follower.pose();
    }
}
