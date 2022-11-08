package org.firstinspires.ftc.teamcode.mechwarriors.bkup;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;

public class EthanClaw implements Claw {

    Servo leftClawServo;
    Servo rightClawServo;

    public EthanClaw(HardwareMap hardwareMap) {
        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(0);
    }

    @Override
    public void open() {
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(0);
    }

    @Override
    public void close() {
        leftClawServo.setPosition(1.0);
        rightClawServo.setPosition(1.0);
    }
}
