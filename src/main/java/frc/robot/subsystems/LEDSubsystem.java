package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private Spark lightController;
    private String color;

    public LEDSubsystem(int portNum) {
        lightController = new Spark(portNum);
        setFixedWhite(); // Default to white
    }

    public void setYellow() {
        lightController.set(0.69);  // Fixed palette yellow
        color = "Yellow";
    }

    public void setPurple() {
        lightController.set(0.91);  // Fixed palette purple
        color = "Purple";
    }

    public void setFixedWhite() {
        lightController.set(0.93);  // Fixed palette white
        color = "White";
    }

    public void setStrobeGreen() {
        // Options for darker/more vibrant green:
        lightController.set(0.77);  // Solid Dark Green
        color = "Strobe Green";
    }

    public void setFixedGreen() {
        lightController.set(0.77);  // Fixed palette green
        color = "Fixed Green";
    }

    public String getColor() {
        return color;
    }
} 