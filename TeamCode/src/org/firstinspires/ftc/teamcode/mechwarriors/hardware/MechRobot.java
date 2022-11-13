package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class MechRobot {

    private final static double LIFT_SPOOL_DIAMETER_MM = 34;
    private final static double LIFT_SPOOL_DIAMETER_IN = LIFT_SPOOL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double LIFT_SPOOL_CIRCUMFERENCE_IN = LIFT_SPOOL_DIAMETER_IN * Math.PI;
    private final static double LIFT_MOTOR_TICKS_PER_ROTATION = 28;
    private final static double LIFT_MOTOR_GEAR_RATIO = 20;
    private final static double LIFT_SPOOL_TICKS_PER_ROTATION = LIFT_MOTOR_GEAR_RATIO * LIFT_MOTOR_TICKS_PER_ROTATION;
    private final static double LIFT_SPOOL_TICKS_PER_ONE_INCH = LIFT_SPOOL_CIRCUMFERENCE_IN / LIFT_SPOOL_TICKS_PER_ROTATION;

    // Drive motors: 537.7 ticks per revolution
    // Wheels 96mm diameter
    private final static double DRIVE_WHEEL_DIAMETER_MM = 96;
    private final static double DRIVE_WHEEL_DIAMETER_IN = DRIVE_WHEEL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double DRIVE_WHEEL_CIRCUMFERENCE_IN = DRIVE_WHEEL_DIAMETER_IN * Math.PI;
    private final static double DRIVE_WHEEL_TICKS_PER_ROTATION = 537.7;
    private final static double DRIVE_WHEEL_TICKS_PER_ONE_INCH = DRIVE_WHEEL_CIRCUMFERENCE_IN / DRIVE_WHEEL_TICKS_PER_ROTATION;


    private final static double LIFT_MAX_UP_POWER = 0.5;
    private final static double LIFT_MAX_DOWN_POWER = 0.5;
    private final static double LIFT_MIN_TICKS = 0;
    private final static double LIFT_MAX_TICKS = 4500;
    private final static double LIFT_SLOW_ZONE = 500;


    // Drive motors
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    // Lift motors
    DcMotor leftLiftMotor;
    DcMotor rightLiftMotor;

    // IMU
    BNO055IMU imu;

    Claw claw;

    public MechRobot(HardwareMap hardwareMap) {
        // Front Left Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Front Right Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");

        // Back Left Motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Back Right Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetMotorTicks();

        // Left Lift Motor
        leftLiftMotor = hardwareMap.get(DcMotor.class, "left_lift_motor");
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Right Lift Motor
        rightLiftMotor = hardwareMap.get(DcMotor.class, "right_lift_motor");
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initIMU(hardwareMap);

        claw = new EthanClaw(hardwareMap);
    }

    void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    public Claw getClaw() {
        return claw;
    }

    public void drive(double powerFrontRight, double powerFrontLeft, double powerBackLeft, double powerBackRight) {
        frontRightMotor.setPower(powerFrontRight);
        frontLeftMotor.setPower(powerFrontLeft);
        backLeftMotor.setPower(powerBackLeft);
        backRightMotor.setPower(powerBackRight);
    }

    public void mecanumDrive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontRightPower = ((y - x - rx) / denominator);
        double frontLeftPower = ((y + x + rx) / denominator);
        double backLeftPower = ((y - x + rx) / denominator);
        double backRightPower = ((y + x - rx) / denominator);
        drive(frontRightPower, frontLeftPower, backLeftPower, backRightPower);
    }

    /**
     * Computes the number of drive motor ticks to go the specified distance
     *
     * @param distanceInInches distance in inches
     * @return number of ticks
     */
    public double calculateDriveTicks(double distanceInInches) {
        return distanceInInches / DRIVE_WHEEL_TICKS_PER_ONE_INCH;
    }

    /**
     * Returns the distance the robot has traveled forward or backward in ticks
     *
     * @return the average ticks
     */
    public double getDriveTicks() {
        // use getCurrentPosition method
        double motorTicks = (frontLeftMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4.0;
        //System.out.println("motor ticks: " + motorTicks);
        return motorTicks;
    }

    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void liftArmUp() {
        if (leftLiftMotor.getCurrentPosition() >= LIFT_MAX_TICKS || rightLiftMotor.getCurrentPosition() >= LIFT_MAX_TICKS) {
            liftArmStop();
        } else if (leftLiftMotor.getCurrentPosition() >= (LIFT_MAX_TICKS - LIFT_SLOW_ZONE) || rightLiftMotor.getCurrentPosition() >= (LIFT_MAX_TICKS - LIFT_SLOW_ZONE)) {
            leftLiftMotor.setPower(LIFT_MAX_UP_POWER * 0.5);
            rightLiftMotor.setPower(LIFT_MAX_UP_POWER * 0.5);
        } else {
            leftLiftMotor.setPower(LIFT_MAX_UP_POWER);
            rightLiftMotor.setPower(LIFT_MAX_UP_POWER);
        }
    }

    public void liftArmDown() {
        if (leftLiftMotor.getCurrentPosition() <= LIFT_MIN_TICKS || rightLiftMotor.getCurrentPosition() <= LIFT_MIN_TICKS) {
            liftArmStop();
        } else if (leftLiftMotor.getCurrentPosition() <= LIFT_SLOW_ZONE || rightLiftMotor.getCurrentPosition() <= LIFT_SLOW_ZONE) {
            leftLiftMotor.setPower(-LIFT_MAX_DOWN_POWER * 0.5);
            rightLiftMotor.setPower(-LIFT_MAX_DOWN_POWER * 0.5);
        } else {
            leftLiftMotor.setPower(-LIFT_MAX_DOWN_POWER);
            rightLiftMotor.setPower(-LIFT_MAX_DOWN_POWER);
        }
    }

    public void liftArmStop() {
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
    }

    public double getLiftTicks() {
        return (leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2.0;
    }

    public double calculateLiftTicks(double heightInInches) {
        return heightInInches / LIFT_SPOOL_TICKS_PER_ONE_INCH;
    }

    public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void resetMotorTicks() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop() {
        this.drive(0, 0, 0, 0);
    }
}
