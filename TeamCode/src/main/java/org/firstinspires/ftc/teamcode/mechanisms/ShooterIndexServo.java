package org.firstinspires.ftc.teamcode.mechanisms;

public class ShooterIndexServo {
    private final double  servoInPosition = .5;
    private final double servoOutPosition = 0.05;

    private double servoPosition;

    private final double timeIn = 200;
    private final double timeOut = 200;

    private CountDownTimer countDownTimer;

    public void defaultStateReset() {
        StateClass.setShooterServoState(StateClass.ShooterServoState.OUT);
    }

    public ShooterIndexServo() {
        countDownTimer = new CountDownTimer();
    }

    public void servoOut() {
        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.IN || StateClass.getShooterServoState() == StateClass.ShooterServoState.MOVING_IN) {
            countDownTimer.setTime(timeOut);
            changed = true;
        }
        else {
            changed = false;
        }
        StateClass.setShooterServoState(StateClass.ShooterServoState.MOVING_OUT);
        servoPosition = servoOutPosition;
    }

    public void servoIn() {
        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.OUT || StateClass.getShooterServoState() == StateClass.ShooterServoState.MOVING_OUT) {
            countDownTimer.setTime(timeIn);
            changed = true;
        }
        else {
            changed = false;
        }
        StateClass.setShooterServoState(StateClass.ShooterServoState.MOVING_IN);
        servoPosition = servoInPosition;
    }

    private boolean changed;

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
