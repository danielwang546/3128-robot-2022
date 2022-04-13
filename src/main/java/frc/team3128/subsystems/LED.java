package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3128.Constants.LEDConstants.*;

public class LED extends SubsystemBase {

    public enum LEDState {
        RAINBOW,
        RED,
        BLUE,
        SCANNER,
        CHASE,
        COLORWIPE_GREEN,
        HORSE_RACE
    }

    private static LED instance;
    private LEDState state = LEDState.HORSE_RACE;

    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    int m_rainbowFirstPixelHue = 0;

    int eyePos = 0;
    int scanDirection = 1;

    double zeroPos = 0;

    int wipeCurrPos = 0;

    int horseRacePos = 0;

    public LED() {
        ledStrip = new AddressableLED(LED_PORT);

        ledBuffer = new AddressableLEDBuffer(LENGTH);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();
    }

    public static synchronized LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    public void setState(LEDState state) {
        this.state = state;   
    }

    private void rainbow() {
        for (int i = 0; i < LENGTH; i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / LENGTH)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
        
    }

    private void solidColor(Color color) {
        for (int i = 0; i < LENGTH; i++) {
            ledBuffer.setLED(i, color);
        }
        
        System.out.println("kanvar is super smart");
        System.out.println("kanvar should be el presidente");
        System.out.println("AAron is watching me hack the system");

    }

    private void scanner() {
        for(int i = 0; i < LENGTH; i++) {
            double distFromEye = MathUtil.clamp(Math.abs(eyePos - i), 0, LENGTH - 1);
            double intensity = 1 - (double)distFromEye / LENGTH;
            double red =  MathUtil.interpolate(BACKGROUND_COLOR.red, EYE_COLOR.red, intensity);
            double green = MathUtil.interpolate(BACKGROUND_COLOR.green, EYE_COLOR.green, intensity);
            double blue = MathUtil.interpolate(BACKGROUND_COLOR.blue, EYE_COLOR.blue, intensity);

            ledBuffer.setLED(i, new Color(red, green, blue));
        }
        if (eyePos == 0) {
            scanDirection = 1;
        }
        if (eyePos == LENGTH - 1) {
            scanDirection = -1;
        }

        eyePos += scanDirection;
    }

    private void chase() {
        zeroPos += CHASE_SPEED;

        for (int i = 0; i < LENGTH; i++) {
            double pctDownStrip = (double)i / LENGTH;
            double numCycles = (double)LENGTH / STRIPE_WIDTH / 2;

            double colorBump = 0.5 * Math.sin(2 * Math.PI * numCycles * (pctDownStrip-zeroPos)) + 0.5;

            colorBump *= colorBump;
            colorBump *= 255;

            ledBuffer.setRGB(i, (int)colorBump, (int)colorBump, 255);
        }
    }

    private void colorWipe(Color color) {
        colorWipe(color, Color.kBlack);
    }

    private void colorWipe(Color color1, Color color2) {
        for (int i = 0; i < LENGTH; i++) {
            if (i >= wipeCurrPos && i < wipeCurrPos + LENGTH) {
                ledBuffer.setLED(i, color1);
            } else {
                ledBuffer.setLED(i, color2);
            }
        }

        wipeCurrPos += 1;
        if (wipeCurrPos == LENGTH) {
            wipeCurrPos = -LENGTH;
        }
    }

    private void sevenColorHorseRace() {
        for (int i = 0; i < LENGTH; i++) {
            ledBuffer.setLED(i, Color.kBlack);

            for (int j = 0; j < 7; j++) {

                int idx = (horseRacePos + j * HORSE_RACE_SPACING) % (7 * HORSE_RACE_SPACING);

                if (i >= idx && i < idx + STRIPE_WIDTH) {
                    ledBuffer.setLED(i, HORSE_RACE_COLORS[j]);
                }
            }
        }

        horseRacePos = (horseRacePos + 1) % (7 * HORSE_RACE_SPACING);
        
    }

    @Override
    public void periodic() {

        switch(state) {
            case RAINBOW:
                rainbow();
                break;
            case RED:
                solidColor(Color.kDarkRed);
                break;
            case BLUE:
                solidColor(Color.kRoyalBlue);
                break;
            case SCANNER:
                scanner();
                break;
            case CHASE:
                chase();
                break;
            case COLORWIPE_GREEN:
                colorWipe(Color.kGreen);
                break;
            case HORSE_RACE:
                sevenColorHorseRace();
                break;
        }

        SmartDashboard.putString("LED State", state.toString());

        ledStrip.setData(ledBuffer);
    }

}