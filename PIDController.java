package frc.robot;

public class PIDController {
    private double kP; 
    private double kI; 
    private double kD; 
    private double setpoint; 
    private double currentError;
    private double previousError; 
    private double integral; 
    private double derivative; 

    public PIDController(double kP, double kI, double kD) {
        // This method sets the kP, kI, and kD variables to the values received by the method. 
        this.kP = kP; 
        this.kI = kI; 
        this.kD = kD; 
    }

    public void setSetpoint(double setpoint) {
        // This method lets you change the setpoint (desired outcome). 
        this.setpoint = setpoint; 
    }

    public double calculate(double measuredSystemState) {
        // This method calculates the PID Controller's output by recieving the current measured system state and plugging values into the formula. 
        currentError = setpoint - measuredSystemState; 
        integral += currentError * 0.02; // Multiplied by 0.02 because of the 20ms ticks. 
        derivative = (currentError - previousError) / 0.02; 
        previousError = currentError; 
        return (kP * currentError) + (kI * integral) + (kD * derivative); 
    }
}