package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp
public class ArmTest extends LinearOpMode {

    @Config
    public static class ArmTestPos {
        public static double test_pos = 0.25;

    }

    @Override
    public void runOpMode() {
        Arm arm = new Arm(this, hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                arm.arm.setPosition(ArmTestPos.test_pos);
            }
        }
    }
}
