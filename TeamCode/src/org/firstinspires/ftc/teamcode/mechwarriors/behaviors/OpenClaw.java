package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;

public class OpenClaw extends Behavior {
    Claw claw;
    ElapsedTime timer;

    public OpenClaw(Telemetry telemetry, String name, Claw claw) {
        this.telemetry = telemetry;
        this.name = name;
        this.claw = claw;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void start() {
        this.claw.open();
        timer.reset();
    }

    @Override
    public void run() {
        if (timer.milliseconds() > 1000) {
            telemetry.addData("Opening Claw", timer.milliseconds());
            this.isDone = true;
        }
    }
}
