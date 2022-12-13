package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class RaiseLift extends Behavior {
    MechRobot robot;
    JunctionType junctionType;
    int ticks;

    public RaiseLift(Telemetry telemetry, String name, MechRobot robot, JunctionType junctionType) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.junctionType = junctionType;
        this.name = name;

        switch (this.junctionType) {
            case GROUND:
                ticks = 0;
                break;
            case TRAVEL:
                ticks = 300;
                break;
            case LOW:
                ticks = 500;
                break;
            case MEDIUM:
                ticks = 1700;
                break;
            case HIGH:
                ticks = 4000;
                break;
        }
    }

    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        if (robot.getLift().getLiftTicks() < ticks) {
            robot.getLift().liftArmUp();
        } else {
            robot.getLift().liftArmStop();
            this.isDone = true;
        }
    }
}
