package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterIndexServo {
    public static double  servoInPosition = .75;
    public static double servoOutPosition = .61;

    private double servoPosition;

    private final double timeIn = 110;//65;
    private final double timeOut = 110;//65;

    private CountDownTimer countDownTimer;
    private boolean changed = true;

    public ShooterIndexServo() {
        countDownTimer = new CountDownTimer();
    }

    public void defaultStateReset() {
        servoOut();
        StateClass.setShooterServoState(StateClass.ShooterServoState.OUT);
    }

    public void servoOut() {
        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.IN || StateClass.getShooterServoState() == StateClass.ShooterServoState.MOVING_IN) {
            countDownTimer.setTime(timeOut);
            changed = true;
            StateClass.setShooterServoState(StateClass.ShooterServoState.MOVING_OUT);

        }
        else {
            changed = false;
        }
        servoPosition = servoOutPosition;
    }

    public void servoIn() {
        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.OUT || StateClass.getShooterServoState() == StateClass.ShooterServoState.MOVING_OUT) {
            countDownTimer.setTime(timeIn);
            changed = true;
            StateClass.setShooterServoState(StateClass.ShooterServoState.MOVING_IN);
        }
        else {
            changed = false;
        }
        servoPosition = servoInPosition;
    }

    public boolean wasChanged() {
        return changed;
    }

    public void checkServoTimer() {
        if (countDownTimer.timeElapsed()) {
            switch (StateClass.getShooterServoState()) {
                case MOVING_IN:
                    StateClass.setShooterServoState(StateClass.ShooterServoState.IN);
                    break;
                case MOVING_OUT:
                    StateClass.setShooterServoState(StateClass.ShooterServoState.OUT);
                    break;
            }
        }
    }

    public double getServoPosition() {
        return servoPosition;
    }
}
