package org.firstinspires.ftc.teamcode.mechanisms;

public class Shooter {
    private double shooterSpeed;
    double threshold = 50;
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
        shooterSpeed = -.7;
    }

    public void updateShooterState(double shooterVelo) {
        if (StateClass.getShooterState() != StateClass.ShooterState.STOPPED) {
            percentError = (shooterVelo - shooterSpeed) / shooterSpeed * 100;
            if (Math.abs(percentError) < threshold) {
                StateClass.setShooterState(StateClass.ShooterState.ATSPEED);
            } else {
                StateClass.setShooterState(StateClass.ShooterState.WINDINGUP);
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
