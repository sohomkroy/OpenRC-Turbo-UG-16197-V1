//package org.firstinspires.ftc.teamcode.mechanisms;
//
//public class ShooterIndexServo {
//    private final double  servoInPosition = .6;
//    private final double servoOutPosition = .4;
//
//    private double servoPosition;
//
//    private final double timeIn = 200;
//    private final double timeOut = 200;
//
//    private CountDownTimer countDownTimer;
//
//    public void defaultStateReset() {
//        StateClass.setShooterServoState(StateClass.ShooterServoState.OUT);
//    }
//
//    public ShooterIndexServo() {
//        countDownTimer = new CountDownTimer();
//    }
//
//    public void servoUp() {
//        if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.DOWN || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_DOWN) {
//            countDownTimer.setTime(timeUp);
//            changed = true;
//        }
//        else {
//            changed = false;
//        }
//        StateClass.setServoRaiserState(StateClass.ServoRaiserState.MOVING_UP);
//        servoPosition = servoInPosition;
//    }
//
//    public void servoDown() {
//        if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.UP || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_UP) {
//            countDownTimer.setTime(timeDown);
//            changed = true;
//        }
//        else {
//            changed = false;
//        }
//        StateClass.setServoRaiserState(StateClass.ServoRaiserState.MOVING_DOWN);
//        servoPosition = servoOutPosition;
//    }
//
//    private boolean changed;
//
//    public boolean wasChanged() {
//        return changed;
//    }
//
//    public void checkServoTimer() {
//        if (countDownTimer.timeElapsed()) {
//            switch (StateClass.getServoRaiserState()) {
//                case MOVING_UP:
//                    StateClass.setServoRaiserState(StateClass.ServoRaiserState.UP);
//                    break;
//                case MOVING_DOWN:
//                    StateClass.setServoRaiserState(StateClass.ServoRaiserState.DOWN);
//                    break;
//            }
//        }
//    }
//
//    public double getServoPosition() {
//        return servoPosition;
//    }
//}
