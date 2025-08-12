package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CenterStagePracticeOpMode extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "left_claw_servo");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("ticks per rev", frontLeftMotor.getMotorType().getTicksPerRev());

    }


    @Override
    public void loop() {
    double speed = gamepad1.left_stick_y;
    frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);


    int ticks = frontLeftMotor.getCurrentPosition();
    telemetry.addData("currentleftmotorposition", ticks);

    if(gamepad1.a){
        servo.setPosition(1.0);
    } else {
        servo.setPosition(0);
    }
    }
}
