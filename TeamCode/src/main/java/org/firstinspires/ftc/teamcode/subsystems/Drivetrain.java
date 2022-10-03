package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Want to toggle between this and normal driving

public class Drivetrain {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    public LinearOpMode l;
    public Telemetry realTelemetry;
    public BNO055IMU    imu;

    private boolean inputButtonPressed;

    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class);

    @Config
    public static class TeleOpDTConstants {
        //Biases so we don't go too fast
        public static double turning_modifier = 0.70;
//        public static double y_modifier = 0.95;
//        public static double x_modifier = 0.85;
        public static double speedFactor = 0.8;
        public static double power_modifier = Math.sqrt(2);

    }


    public Drivetrain(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu.initialize(parameters);


//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        l.idle();
    }

    //This is the teleop drive formulas
    public void JoystickMovement(double leftStickY, double leftStickX, double rightStickX, double rightstickY, boolean slowModeControl){

//        double RightStickAngle;
        double LeftStickAngle;
        double slowModeMult = slowModeControl ? 0.3 : 1;

        //Angles measured from (0,1) and go clockwise (I know this stinks)

        if (leftStickX == 0 && leftStickY == 0) {
            LeftStickAngle = 0;
        } else {
            LeftStickAngle = Math.atan2(leftStickY, -leftStickX)-Math.PI/4;
        }
//        if (rightStickX == 0) {
//             RightStickAngle = 0;
//        } else {
//            RightStickAngle = Math.atan2(rightstickY, rightStickX);
//        }
        double RobotAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double NewLeftAngle = LeftStickAngle - RobotAngle;
//        double NewRightAngle = RightStickAngle - RobotAngle;
        //Sets motor values based on adding and subtracting joystick values
        double LeftX = Math.cos(NewLeftAngle) * Math.sqrt(Math.pow(leftStickY, 2.0) + Math.pow(leftStickX, 2.0));
        double LeftY = Math.sin(NewLeftAngle) * Math.sqrt(Math.pow(leftStickY, 2.0) + Math.pow(leftStickX, 2.0));
//        double RightX = Math.cos(NewRightAngle) * Math.sqrt(Math.pow(rightStickX, 2.0) + Math.pow(rightstickY, 2.0));

        double RightX = rightStickX;
        double frontLeftVal = (- RightX) + LeftX;
        double frontRightVal = (LeftY + RightX) ;
        double backLeftVal = ((LeftY - RightX) );
        double backRightVal = (( RightX) + LeftX);



        frontLeftDrive.setPower(frontLeftVal * slowModeMult * TeleOpDTConstants.power_modifier);
        frontRightDrive.setPower(frontRightVal * slowModeMult * TeleOpDTConstants.power_modifier);
        backLeftDrive.setPower(backLeftVal * slowModeMult * TeleOpDTConstants.power_modifier);
        backRightDrive.setPower(backRightVal * slowModeMult * TeleOpDTConstants.power_modifier);

        realTelemetry.addData("left stick angle", LeftStickAngle);
//        realTelemetry.addData("right stick angle", RightStickAngle);
        realTelemetry.addData("imu angle 1", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        realTelemetry.addData("imu angle 2", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        realTelemetry.addData("imu angle 3", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        realTelemetry.addData("FLV", frontLeftVal);
        realTelemetry.addData("LeftX", LeftX);
        realTelemetry.addData("RightX", RightX);
        realTelemetry.addData("LeftY", LeftY);
        realTelemetry.addData("Robot Angle", RobotAngle);


    }

//    double cubeInput (double input, double factor) {
//        double t = factor * Math.pow(input,3 );
//        double r = input * (1 - factor);
//        return t + r;
//
//    }
}