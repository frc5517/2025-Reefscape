package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class AddressableLEDSubsystem extends SubsystemBase {

    private AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private LEDPattern pattern;

    private final LEDPattern rainbow =
        LEDPattern.rainbow(255, Constants.AddressableConstants.kLedLength);

    public AddressableLEDSubsystem() {

        led = new AddressableLED(Constants.AddressableConstants.kLedPort);
        buffer = new AddressableLEDBuffer(Constants.AddressableConstants.kLedLength);

        led.setLength(buffer.getLength());

        led.setData(buffer);
        led.start();

    }

    @Override
    public void periodic() {
        pattern.applyTo(buffer);
    }

    public Color editColor(Color color) {
        return new Color(color.green, color.red, color.blue);
    }

    public void setSolidColor(Color color) {
        pattern = LEDPattern.solid(color);
    }

    public void setProgressMask(Color progressColor, Color defaultColor, double progress, double maxProgress) {
        pattern = LEDPattern.progressMaskLayer(() -> progress / maxProgress);
    }

    public void scrollingRainbow(double scrollMetersPerSecond) {
        pattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1),
                Constants.AddressableConstants.kLedSpacing);
    }

    // Will add as I decide how exactly to set up the LEDs

}
