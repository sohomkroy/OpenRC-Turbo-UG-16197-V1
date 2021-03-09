package org.firstinspires.ftc.teamcode.mechanisms;

public class Shooter {
    private double shooterSpeed;
    public void defaultStateReset() {
        StateClass.setShooterState(StateClass.ShooterState.STOPPED);
    }
    public Shooter() {

    }
    public double getShooterSpeed() {
        return shooterSpeed;
    }
    public void setShooterSpeed(double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
    }
    double threshold = .01;
    public void updateShooter(double shooterVelo) {
        switch (StateClass.getShooterState()) {
            case STOPPED:
                shooterSpeed = 0;
                break;
            case WINDINGUP:
                if (Math.abs(shooterVelo-shooterSpeed)/shooterSpeed > threshold) {
                    StateClass.setShooterState(StateClass.ShooterState.ATSPEED);
                }
                break;
            case ATSPEED:
                break;
        }

    }



}
