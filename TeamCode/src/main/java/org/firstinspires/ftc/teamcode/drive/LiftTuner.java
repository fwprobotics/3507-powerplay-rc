package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift2;
import org.firstinspires.ftc.teamcode.subsystems.Lift3;

@TeleOp
@Config
public class LiftTuner extends LinearOpMode {
    public static int height = 10;
    public void runOpMode() {
        Lift3 lift = new Lift3(hardwareMap, telemetry, gamepad2);

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
