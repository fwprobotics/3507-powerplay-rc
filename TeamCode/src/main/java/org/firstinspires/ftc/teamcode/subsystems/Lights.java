package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lights {
    RevBlinkinLedDriver blinkinLedDriver;
    public Lights(HardwareMap hardwareMap, Telemetry telemetry, Field.autoZones zone) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        setZone(zone);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLedDriver.setPattern(pattern);
    }

    public void setZone(Field.autoZones zone) {


    }

}
