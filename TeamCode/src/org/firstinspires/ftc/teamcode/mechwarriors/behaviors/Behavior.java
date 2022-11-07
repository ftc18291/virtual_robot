package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

public abstract class Behavior {
    String name;
    boolean isDone = false;

    public abstract void start();
    public abstract void run();

    public String getName() {
        return name;
    }

    public boolean isDone() {
        return isDone;
    }
}
