package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;

public class CloseClaw extends Behavior {
    Claw claw;
    ElapsedTime timer;

    public CloseClaw(Telemetry telemetry, String name, Claw claw) {
        this.telemetry = telemetry;
        this.name = name;
        this.claw = claw;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void start() {
        this.claw.close();
        timer.reset();
    }

    @Override
    public void run() {
        if (timer.milliseconds() > 2000) {
            telemetry.addData("Closing Claw", timer.milliseconds());
            this.isDone = true;
        }
    }
}
