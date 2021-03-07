package org.firstinspires.ftc.teamcode.mechanisms;

public class Differential {
    private double motor1Power;
    private double motor2Power;

    private double motor1PowerPrev;
    private double motor2PowerPrev;

    private double intakeSpeed;
    private double turretSpeed;

    public Differential() {
        motor1Power = 0;
        motor2Power = 0;
    }

    public void setIntakeSpeed(double speed) {
        if (Math.abs(speed)+Math.abs(turretSpeed) > 2) {
            //throw error / warning
        }
       else {
            if (Math.abs(speed)+Math.abs(turretSpeed) > 1.8) {
                //throw error / warning
            }
            this.intakeSpeed = speed;
            updateSpeeds();
        }
    }

    private void updateSpeeds() {
        motor1PowerPrev = motor1Power;
        motor2PowerPrev = motor2Power;
        motor1Power = (intakeSpeed+turretSpeed)/2.0;
        motor2Power = (intakeSpeed-turretSpeed)/2.0;
    }

    public void setTurretSpeed(double speed) {
        if (Math.abs(speed)+Math.abs(intakeSpeed) > 2) {
            //throw error / warning
        }
        else {
            if (Math.abs(speed)+Math.abs(turretSpeed) > 1.8) {

            }
            this.turretSpeed = speed;
            updateSpeeds();
        }
    }
    private boolean changed;
    private double changedThreshold = .05;

    public boolean wasChanged() {
        //return true;
        if (Math.abs(motor1Power-motor1PowerPrev) > changedThreshold || Math.abs(motor2Power-motor2PowerPrev) > changedThreshold) {
            changed = true;
        }
        else {
            changed = false;
        }
        return changed;
    }

    public double getMotor1Power() {
        return this.motor1Power;
    }

    public double getMotor2Power() {
        return motor2Power;
    }

    public double getIntakeSpeed() {
        return intakeSpeed;
    }

    public double getTurretSpeed() {
        return turretSpeed;
    }
}
