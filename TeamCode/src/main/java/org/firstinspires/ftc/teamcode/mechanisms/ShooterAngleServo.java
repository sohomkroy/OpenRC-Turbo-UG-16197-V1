package org.firstinspires.ftc.teamcode.mechanisms;

public class ShooterAngleServo {
    public final double initialPosition = .5;
    public double servoPosition;

    ShooterAngleServo() {
        this.servoPosition = initialPosition;
    }
    private boolean isChanged;

    public void setServoPosition(double servoPosition) {
        if (servoPosition == this.servoPosition) {
            isChanged = false;
        }
        else {
            this.servoPosition = servoPosition;
            isChanged = true;
        }
    }
    public double getServoPosition() {
        return servoPosition;
    }
    public boolean isChanged() {
        return isChanged;
    }
}
