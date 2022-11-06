package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MechRobot {

    private final static double MILLIMETERS_PER_INCH = 25.4;

    private final static double LIFT_SPOOL_DIAMETER_MM = 34;
    private final static double LIFT_SPOOL_DIAMETER_IN = LIFT_SPOOL_DIAMETER_MM / MILLIMETERS_PER_INCH;
    private final static double LIFT_SPOOL_CIRCUMFERENCE_IN = LIFT_SPOOL_DIAMETER_IN * Math.PI;
    private final static double LIFT_MOTOR_TICKS_PER_ROTATION = 28;
    private final static double LIFT_MOTOR_GEAR_RATIO = 20;
    private final static double LIFT_SPOOL_TICKS_PER_ROTATION = LIFT_MOTOR_GEAR_RATIO * LIFT_MOTOR_TICKS_PER_ROTATION;
    private final static double LIFT_SPOOL_TICKS_PER_ONE_INCH = LIFT_SPOOL_CIRCUMFERENCE_IN / LIFT_SPOOL_TICKS_PER_ROTATION;

    private final static double LIFT_MAX_UP_POWER = 0.5;
    private final static double LIFT_MAX_DOWN_POWER = 0.5;

    // Drive motors
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    // Lift motors
    DcMotor leftLiftMotor;
    DcMotor rightLiftMotor;

    Claw claw;

    MechRobot(HardwareMap hardwareMap) {
        // Front Left Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Front Right Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Back Left Motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Back Right Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        claw = new EthanClaw(hardwareMap);
    }

    Claw getClaw() {
        return claw;
    }

    void drive(double powerFrontRight, double powerFrontLeft, double powerBackLeft, double powerBackRight) {
        frontRightMotor.setPower(powerFrontRight);
        frontLeftMotor.setPower(powerFrontLeft);
        backLeftMotor.setPower(powerBackLeft);
        backRightMotor.setPower(powerBackRight);
    }

    void mecanumDrive(double x, double y, double rx) {
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
    double calculateDriveTicks(double distanceInInches) {
        // TODO: Implement logic
        return 0.0;
    }

    /**
     * Returns the distance the robot has traveled forward or backward in ticks
     *
     * @return the average ticks
     */
    double getDriveTicks() {
        // TODO: Implement - calculate and return the average ticks for all drive motors
        return 0.0;
    }

    public void liftArmUp() {
        // TODO: Stop the lift when it gets to the max extension
        leftLiftMotor.setPower(LIFT_MAX_UP_POWER);
        rightLiftMotor.setPower(LIFT_MAX_UP_POWER);
    }

    public void liftArmDown() {
        // TODO: Stop the lift when it gets to the bottom
        leftLiftMotor.setPower(-LIFT_MAX_DOWN_POWER);
        rightLiftMotor.setPower(-LIFT_MAX_DOWN_POWER);
    }

    public void liftArmStop() {
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
    }

    /**
     *
     * @param junctionType type of junction
     */
    public void liftArmToJunctionHeight(JunctionType junctionType) {
        // TODO: Implement
        switch (junctionType) {
            case GROUND:
                break;
            case LOW:
                break;
            case MEDIUM:
                break;
            case HIGH:
                break;
            default:
        }
    }

    double calculateLiftTicks(double heightInInches) {
        return heightInInches / LIFT_SPOOL_TICKS_PER_ONE_INCH;
    }
}
