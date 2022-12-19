package org.firstinspires.ftc.teamcode.autonomous.lowpole;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Field;


@Autonomous
public class RightRedLowPoleFSM extends LinearOpMode {
    public void runOpMode() {
        LowPoleFSM auto =  new LowPoleFSM(Field.autoZones.REDRIGHT);
        auto.runOpMode();
    }

}
