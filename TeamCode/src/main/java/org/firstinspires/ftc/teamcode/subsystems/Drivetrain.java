package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
// Want to toggle between this and normal driving

public class Drivetrain {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private SampleMecanumDrive drive;

    public LinearOpMode l;
    public Telemetry realTelemetry;
    public BNO055IMU    imu;

    private boolean inputButtonPressed;
    public ToggleButton fieldRelativeDrive;


    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class);

    @Config
    public static class TeleOpDTConstants {
        //Biases so we don't go too fast
        public static double turning_modifier = 0.70;
//        public static double y_modifier = 0.95;
//        public static double x_modifier = 0.85;
        public static double speedFactor = 0.8;
        public static double power_modifier = 0.6;
        public static double lift_up_modifier = 0.2;

    }


    public Drivetrain(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry, SampleMecanumDrive drive){

        l = Input;
        this.drive = drive;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);
        fieldRelativeDrive = new ToggleButton(false);

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

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        l.idle();
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    //This is the teleop drive formulas
    public void JoystickMovement(double leftStickY, double leftStickX, double rightStickX, double rightstickY, boolean slowModeControl, boolean fieldRelativeToggle, boolean boostButton){

//        double RightStickAngle;

        //toggles between field relative drive(aka forward is your forward(default) vs forward is robot forward)
        fieldRelativeDrive.toggle(fieldRelativeToggle);
        double frontRightVal;
        double frontLeftVal;
        double backLeftVal;
        double backRightVal;

        double slowModeMult = slowModeControl ? 0.3 : 1;
        double boostModeMult = boostButton ? 1.5 : 1;


        if (fieldRelativeDrive.state()) {

            double LeftStickAngle;


            //Angles measured from (0,1) and go clockwise (I know this stinks)

            if (leftStickX == 0 && leftStickY == 0) {
                LeftStickAngle = 0;
            } else {
                LeftStickAngle = Math.atan2(leftStickY, -leftStickX) - Math.PI / 4;
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
            frontLeftVal = (-RightX) + LeftX;
            frontRightVal = (LeftY + RightX);
            backLeftVal = ((LeftY - RightX));
            backRightVal = ((RightX) + LeftX);

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

        } else {

            double LeftY = -leftStickY;
            double LeftX = -leftStickX;
            double RightX = -rightStickX;
            if (slowModeControl) {
                frontLeftVal = clamp(cubeInput(((LeftY - RightX) - LeftX), TeleOpDTConstants.speedFactor), -1, 1);
                frontRightVal = clamp(cubeInput(((LeftY + RightX) + LeftX), TeleOpDTConstants.speedFactor), -1, 1);
                backLeftVal = clamp(cubeInput(((LeftY - RightX) + LeftX), TeleOpDTConstants.speedFactor), -1, 1);
                backRightVal = clamp(cubeInput(((LeftY + RightX) - LeftX), TeleOpDTConstants.speedFactor), -1, 1);
            } else {
                frontLeftVal = cubeInput(((LeftY - RightX) - LeftX), TeleOpDTConstants.speedFactor);
                frontRightVal = cubeInput(((LeftY + RightX) + LeftX), TeleOpDTConstants.speedFactor);
                backLeftVal = cubeInput(((LeftY - RightX) + LeftX), TeleOpDTConstants.speedFactor);
                backRightVal = cubeInput(((LeftY + RightX) - LeftX), TeleOpDTConstants.speedFactor);
            }
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -leftStickY,
//                            -leftStickX,
//                            -rightStickX
//                    )
//            );
//            drive.update();

         //   realTelemetry.addData("LeftX", LeftX);
          //  realTelemetry.addData("RightX", RightX);
           // realTelemetry.addData("LeftY", LeftY);
        }

        frontLeftDrive.setPower(frontLeftVal * slowModeMult * boostModeMult * TeleOpDTConstants.power_modifier);
        frontRightDrive.setPower(frontRightVal * slowModeMult * boostModeMult* TeleOpDTConstants.power_modifier);
        backLeftDrive.setPower(backLeftVal * slowModeMult * boostModeMult * TeleOpDTConstants.power_modifier);
        backRightDrive.setPower(backRightVal * slowModeMult  * boostModeMult  * TeleOpDTConstants.power_modifier);

        realTelemetry.addData("Front Left", frontLeftDrive.getCurrentPosition());
        realTelemetry.addData("Back Left", backLeftDrive.getCurrentPosition());
        realTelemetry.addData("Front Right", frontRightDrive.getCurrentPosition());
        realTelemetry.addData("Back Right", backRightDrive.getCurrentPosition());
        realTelemetry.addData("Toggle Field Relative", fieldRelativeDrive.state());
        realTelemetry.addData("Slow Mode Multiplier", slowModeMult);


    }

    double cubeInput (double input, double factor) {
        double t = factor * Math.pow(input,3 );
        double r = input * (1 - factor);
        return t + r;

    }

}