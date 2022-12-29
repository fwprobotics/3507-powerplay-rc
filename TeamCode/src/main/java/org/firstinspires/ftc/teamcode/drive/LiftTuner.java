package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift2;

@TeleOp
public class LiftTuner extends LinearOpMode {
    public void runOpMode() {
        Lift2 lift = new Lift2(hardwareMap, telemetry);

        waitForStart();
        lift.setHeight(10);
        while (opModeIsActive()) {
            lift.update();
        }


    }
}
