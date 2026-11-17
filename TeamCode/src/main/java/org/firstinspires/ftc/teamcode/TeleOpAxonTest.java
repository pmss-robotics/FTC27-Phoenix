package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "Teleop2", group = "Main")
public class TeleOpAxonTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, telemetry);
        double targetPosition = 0.5;
        turret.setTargetPosition(targetPosition);

        telemetry.addData("turret initialized", targetPosition);
        telemetry.update();

        waitForStart();
    }
}
