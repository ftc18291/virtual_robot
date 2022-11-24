package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class LowerLift extends Behavior {
    MechRobot robot;
    JunctionType junctionType;
    int ticks;

    public LowerLift(Telemetry telemetry, String name, MechRobot robot, JunctionType junctionType) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.junctionType = junctionType;
        this.name = name;

        switch (this.junctionType) {
            case GROUND:
                ticks = 100;
                break;
            case LOW:
                ticks = 500;
                break;
            case MEDIUM:
                ticks = 2000;
                break;
            case HIGH:
                ticks = 5000;
                break;
        }
    }


    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        if (robot.getLift().getLiftTicks() > ticks) {
            robot.getLift().liftArmDown();
        } else {
            robot.getLift().liftArmStop();
            this.isDone = true;
        }
    }
}

