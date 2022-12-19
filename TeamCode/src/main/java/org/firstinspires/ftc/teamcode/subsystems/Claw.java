package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//This is quite self explanatory - just moves a servo between positions based on if we want the claw open or not
public class Claw {
    public static Servo ClawServo;
    public static DigitalChannel coneDetector;
    public static Telemetry realTelemetry;

    @Config
    public static class ClawPositions {
        public static double open = 0;
        public static double closed = 1;
        public static double drop = 0.55;
    }

    public enum clawStatuses {
        CLOSED (ClawPositions.closed),
        OPEN (ClawPositions.open),
        DROP (ClawPositions.drop);

        public double position;
        clawStatuses(double position) {this.position = position;}

        public double position() {return position;}
    }

    public Claw(HardwareMap hardwareMap, Telemetry telemetry){
        ClawServo = hardwareMap.servo.get("clawServo");
        coneDetector = hardwareMap.get(DigitalChannel.class, "coneTouch");
        coneDetector.setMode(DigitalChannel.Mode.INPUT);
        realTelemetry = telemetry;
        status = clawStatuses.OPEN;
    }

    public clawStatuses status;

    public void TeleopControl(boolean open, boolean closed, boolean drop){

        if (open) {
            status = clawStatuses.OPEN;
        } else if (closed){
            status = clawStatuses.CLOSED;
        } else if (drop) {
            status = clawStatuses.DROP;
        }

        ClawServo.setPosition(status.position());
        realTelemetry.addData("coneInClaw", coneDetector.getState());
    }

    public void AutoControl(clawStatuses input) {
        ClawServo.setPosition(input.position());
    }


}
