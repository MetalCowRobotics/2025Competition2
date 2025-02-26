package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private Spark lightController;
    private String color;

    public LEDSubsystem(int portNum) {
        lightController = new Spark(portNum);
        setWhite(); // Default to white
    }

    public void setYellow() {
        lightController.set(.69);
        color = "Yellow";
    }

    public void setPurple() {
        lightController.set(.91);
        color = "Purple";
    }

    public void setWhite() {
        lightController.set(.21);
        color = "White";
    }

    public void setStrobeGreen() {
        lightController.set(-0.75); // Strobe green pattern
        color = "Strobe Green";
    }

    public String getColor() {
        return color;
    }
} 