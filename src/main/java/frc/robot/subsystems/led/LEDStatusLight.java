package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class LEDStatusLight extends SubsystemBase {
    private static final int DASHBOARD_DISPLAY_LENGTH = 20;
    private static AddressableLED led = null;
    private final Color[] ledColors;
    private final Color[] dashboardColors;
    private final String[] dashboardColorsHex;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView[] views;

    public LEDStatusLight(int port, int lengthOfEachSide, boolean... viewsReversed) {
        // make sure length is even
        int length = lengthOfEachSide * viewsReversed.length;

        this.ledColors = new Color[lengthOfEachSide];
        this.dashboardColors = new Color[DASHBOARD_DISPLAY_LENGTH];
        this.dashboardColorsHex = new String[DASHBOARD_DISPLAY_LENGTH];
        Arrays.fill(ledColors, new Color());
        Arrays.fill(dashboardColors, new Color());
        this.buffer = new AddressableLEDBuffer(length);

        views = new AddressableLEDBufferView[viewsReversed.length];
        int tmp = 0;
        for (int i = 0; i < viewsReversed.length; i++) {
            views[i] = buffer.createView(tmp, lengthOfEachSide);
            if (viewsReversed[i]) views[i] = views[i].reversed();
            tmp += lengthOfEachSide;
        }

        if (led != null) led.close();
        led = new AddressableLED(port);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        for (AddressableLEDBufferView view : views)
            for (int i = 0; i < ledColors.length; i++) view.setLED(i, ledColors[i]);
        led.setData(buffer);
        for (int i = 0; i < DASHBOARD_DISPLAY_LENGTH; i++) dashboardColorsHex[i] = dashboardColors[i].toHexString();
        Logger.recordOutput("Status Light", dashboardColorsHex);
    }

    public Command playAnimation(LEDAnimation animation, double timeSeconds) {
        Timer timer = new Timer();
        timer.start();
        return this.run(() -> {
                    animation.play(ledColors, timer.get() / timeSeconds);
                    animation.play(dashboardColors, timer.get() / timeSeconds);
                })
                .beforeStarting(timer::reset)
                .withTimeout(timeSeconds)
                .ignoringDisable(true);
    }

    public Command playAnimation(LEDAnimation animation, double timeSeconds, int loopNum) {
        return playAnimation(animation, timeSeconds).repeatedly().withTimeout(timeSeconds * loopNum);
    }

    public Command playAnimationPeriodically(LEDAnimation animation, double hz) {
        double timeSeconds = 1.0 / hz;
        return this.playAnimation(animation, timeSeconds).repeatedly().ignoringDisable(true);
    }

    public Command showEnableDisableState() {
        return new ConditionalCommand(
                        playAnimation(new LEDAnimation.SlideBackAndForth(new Color(0, 200, 255)), 5)
                                .until(RobotState::isDisabled),
                        playAnimation(new LEDAnimation.Breathe(new Color(0, 200, 255)), 3)
                                .until(RobotState::isEnabled),
                        RobotState::isEnabled)
                .repeatedly()
                .ignoringDisable(true);
    }
}
