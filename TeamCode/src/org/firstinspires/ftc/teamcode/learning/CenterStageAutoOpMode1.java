package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

@Autonomous
public class CenterStageAutoOpMode1 extends OpMode {
    private final static double DRIVE_WHEEL_DIAMETER_MM = 96;
    private final static double DRIVE_WHEEL_DIAMETER_IN = DRIVE_WHEEL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double DRIVE_WHEEL_CIRCUMFERENCE_IN = DRIVE_WHEEL_DIAMETER_IN * Math.PI;
    private final static double DRIVE_WHEEL_TICKS_PER_ROTATION = 537.7;
    private final static double DRIVE_WHEEL_TICKS_PER_ONE_INCH = DRIVE_WHEEL_CIRCUMFERENCE_IN / DRIVE_WHEEL_TICKS_PER_ROTATION;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    String state;
    IMU imu;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        state = "driveForward24";
        initIMU(hardwareMap);
    }

    @Override
    public void loop() {
        switch (state) {
            case "driveForward24":
                double ticks1 = calculateDriveTicks(24);
                double distance = frontLeftMotor.getCurrentPosition();
                if (ticks1 > distance) {
                    frontLeftMotor.setPower(1);
                    backLeftMotor.setPower(1);
                    backRightMotor.setPower(1);
                    frontRightMotor.setPower(1);
                } else {
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    state = "turnLeft90";

                }
                break;
            case "turnLeft90":
                double currentHeading = getHeading();
                double desiredHeading = 90;
                telemetry.addData("desiredHeading", desiredHeading);
                telemetry.addData("currentHeading", currentHeading);
                if (currentHeading < desiredHeading) {
                    frontLeftMotor.setPower(-.1);
                    backLeftMotor.setPower(-.1);
                    backRightMotor.setPower(.1);
                    frontRightMotor.setPower(.1);
                } else {
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    state = "end";
                }
                break;
        }

    }

    public int calculateDriveTicks(double distanceInInches) {
        return (int) (distanceInInches / DRIVE_WHEEL_TICKS_PER_ONE_INCH);
    }

    void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
