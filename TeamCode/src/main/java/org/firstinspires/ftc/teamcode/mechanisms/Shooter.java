package org.firstinspires.ftc.teamcode.mechanisms;

public class Shooter {
    private double shooterSpeed;
    double threshold = 5;
    public void defaultStateReset() {
        StateClass.setShooterState(StateClass.ShooterState.STOPPED);
    }
    private double percentError;
    public double getShooterSpeed() {
        return shooterSpeed;
    }
    public void setShooterSpeed(double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
    }
    public Shooter() {
    }
    CountDownTimer shooterTimer = new CountDownTimer();

    public void updateShooterState(double shooterVelo) {
        if (StateClass.getShooterState() != StateClass.ShooterState.STOPPED) {
            percentError = (shooterVelo - shooterSpeed) / shooterSpeed * 100;
            if (percentError>=-1) {
                StateClass.setShooterState(StateClass.ShooterState.ATSPEED);

//                if (shooterTimer.timeElapsed()) {
//                    StateClass.setShooterState(StateClass.ShooterState.ATSPEED);
//                }
            } else {
                StateClass.setShooterState(StateClass.ShooterState.WINDINGUP);
                //shooterTimer.setTime(40);
            }
        }
    }
    public double getPercentError() {
        return percentError;
    }
//    public void updateShooter(double shooterVelo) {
//        switch (StateClass.getShooterState()) {
//            case STOPPED:
//                shooterSpeed = 0;
//                break;
//            case WINDINGUP:
//                if (Math.abs(shooterVelo-shooterSpeed)/shooterSpeed > threshold) {
//                    StateClass.setShooterState(StateClass.ShooterState.ATSPEED);
//                }
//                break;
//            case ATSPEED:
//                break;
//        }
//
//    }
}
