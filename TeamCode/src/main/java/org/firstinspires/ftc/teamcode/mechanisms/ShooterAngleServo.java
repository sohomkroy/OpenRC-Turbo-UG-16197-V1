package org.firstinspires.ftc.teamcode.mechanisms;

public class ShooterAngleServo {
    public final double initialPosition = .60;
    public double servoPosition;
    private boolean isChanged = true;
    public ShooterAngleServo() {

    }

    public void defaultStateRest() {
        setServoPosition(initialPosition);
    }

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
