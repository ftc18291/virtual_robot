package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Arc;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;

/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, two servos for the claw, two motors for the linear slide lift
 * <p>
 * MechWarriorsPowerPlayMecanumBot is the controller class for the "mech_warriors_powerplay_mecanum_bot.fxml" markup file.
 */
@BotConfig(name = "Mech Warriors PowerPlay Bot", filename = "mech_warriors_intothedeep_mecanum_bot")
public class MechWarriorsIntoTheDeepMecanumBot extends MecanumPhysicsBase {

    private ServoImpl leftClawServo = null;
    private ServoImpl rightClawServo = null;

    private DcMotorExImpl leftLiftMotor = null;
    private DcMotorExImpl rightLiftMotor = null;

    // these are instantiated during loading via a fx:id property.
    @FXML
    Rectangle leftClaw;
    @FXML
    Rectangle rightClaw;
    @FXML
    Arc leftLiftMotorCircle;
    @FXML
    Arc rightLiftMotorCircle;

    public MechWarriorsIntoTheDeepMecanumBot() {
        super();
    }

    public void initialize() {
        super.initialize();
        hardwareMap.setActive(true);
        leftClawServo = (ServoImpl) hardwareMap.servo.get("left_claw_servo");
        rightClawServo = (ServoImpl) hardwareMap.servo.get("right_claw_servo");

        leftLiftMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "left_lift_motor");
        rightLiftMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "right_lift_motor");

        hardwareMap.setActive(false);
        leftClaw.getTransforms().add(new Rotate(0, 20, 2.5));
        rightClaw.getTransforms().add(new Rotate(0, 0, 2.5));

        leftLiftMotorCircle.getTransforms().add(new Rotate(0, 0, 0));
        rightLiftMotorCircle.getTransforms().add(new Rotate(0, 0, 0));
    }

    protected void createHardwareMap() {
        super.createHardwareMap();
        hardwareMap.put("left_claw_servo", new ServoImpl());
        hardwareMap.put("right_claw_servo", new ServoImpl());

        hardwareMap.put("left_lift_motor", new DcMotorExImpl(MotorType.RevUltraPlanetaryOneToOne, motorController0, 0));
        hardwareMap.put("right_lift_motor", new DcMotorExImpl(MotorType.RevUltraPlanetaryOneToOne, motorController0, 1));
    }

    public synchronized void updateStateAndSensors(double millis) {
        //Compute new pose and update various sensors
        super.updateStateAndSensors(millis);
        leftLiftMotor.update(millis);
        rightLiftMotor.update(millis);
    }

    public synchronized void updateDisplay() {
        super.updateDisplay();
        ((Rotate) leftClaw.getTransforms().get(0)).setAngle(120.0 * leftClawServo.getPosition() + 90);
        ((Rotate) rightClaw.getTransforms().get(0)).setAngle(-120.0 * rightClawServo.getPosition() - 90);

        ((Rotate) leftLiftMotorCircle.getTransforms().get(0)).setAngle(leftLiftMotor.getActualPosition());
        ((Rotate) rightLiftMotorCircle.getTransforms().get(0)).setAngle(rightLiftMotor.getActualPosition());
    }

    public void powerDownAndReset() {
        super.powerDownAndReset();

    }
}