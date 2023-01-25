package org.firstinspires.ftc.teamcode.autonomous.highpole;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Field;


@Autonomous
public class RightRedFSM extends LinearOpMode {
    public void runOpMode() {
        HighPoleFSM auto =  new HighPoleFSM();
        auto.runOpMode();
    }

}
