package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wait extends Behavior {

    ElapsedTime timer;    // todo: write your code here
    int delayMilliseconds;

    public Wait(Telemetry telemetry, String name, int delayMilliseconds) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.delayMilliseconds = delayMilliseconds;
        this.telemetry = telemetry;
    }

    public void start() {
        timer.reset();
    }

    public void run() {
        if (timer.milliseconds() >= delayMilliseconds) {
            telemetry.addData("Waiting", timer.milliseconds());
            this.isDone = true;
        }
    }
}
