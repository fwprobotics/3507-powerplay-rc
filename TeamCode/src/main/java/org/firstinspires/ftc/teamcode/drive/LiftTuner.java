package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift2;

@TeleOp
@Config
public class LiftTuner extends LinearOpMode {
    public static int height = 15;
    public void runOpMode() {
        Lift2 lift = new Lift2(hardwareMap, telemetry);

        waitForStart();
        lift.setHeight(height);
        while (opModeIsActive()) {
            lift.update();
            if (!lift.isBusy()) {
                lift.setHeight(0);
            }
        }


    }
}
