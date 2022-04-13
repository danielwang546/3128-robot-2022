package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3128.Constants.LEDConstants.*;

/**
 * @author Daniel Wang, Kanvar Soin
 */
public class LED extends SubsystemBase {

    public enum LEDState {
        RAINBOW,
        RED,
        BLUE,
        GREEN,
        ORANGE,
        RED_ALLIANCE,
        BLUE_ALLIANCE,
        SCANNER,
        CHASE,
        COLORWIPE_GREEN,
        HORSE_RACE
    }

    private static LED instance;

    private LEDState topHalfState = LEDState.CHASE;
    private LEDState bottomHalfState = LEDState.CHASE;

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

    public void setTopState(LEDState state) {
        topHalfState = state;   
    }

    public void setBottomState(LEDState state) {
        bottomHalfState = state;
    }

    public void defaultState() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            setTopState(LEDState.RED_ALLIANCE);
            setBottomState(LEDState.RED_ALLIANCE);
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            setTopState(LEDState.BLUE_ALLIANCE);
            setBottomState(LEDState.BLUE_ALLIANCE);
        } else {
            setTopState(LEDState.CHASE);
            setBottomState(LEDState.CHASE);
        }
    }

    private void rainbow(int[] idxs) {
        for (int i : idxs) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / LENGTH)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
        
    }

    private void solidColor(int[] idxs, Color color) {
        for (int i : idxs) {
            ledBuffer.setLED(i, color);
        }
        
        System.out.println("kanvar is super smart");
        System.out.println("kanvar should be el presidente");
        System.out.println("AAron is watching me hack the system");

    }

    private void scanner(int[] idxs) {
        for(int i : idxs) {
            double distFromEye = MathUtil.clamp(Math.abs(eyePos - i), 0, LENGTH - 1);
            double intensity = 1 - (double)distFromEye / LENGTH;

            ledBuffer.setLED(i, cInterp(BACKGROUND_COLOR, EYE_COLOR, intensity));
        }
        if (eyePos == 0) {
            scanDirection = 1;
        }
        if (eyePos == LENGTH - 1) {
            scanDirection = -1;
        }

        eyePos += scanDirection;
    }
    
    private void chase(int[] idxs, Color c1, Color c2) {
        zeroPos += CHASE_SPEED;

        for (int i : idxs) {
            double pctDownStrip = (double)i / LENGTH;
            double numCycles = (double)LENGTH / STRIPE_WIDTH / 2;

            double colorBump = 0.5 * Math.sin(2 * Math.PI * numCycles * (pctDownStrip-zeroPos)) + 0.5;

            colorBump *= colorBump;

            ledBuffer.setLED(i, cInterp(c1, c2, colorBump));
        }
    }

    private void colorWipe(int[] idxs, Color color) {
        colorWipe(idxs, color, Color.kBlack);
    }

    private void colorWipe(int[] idxs, Color c1, Color c2) {
        for (int i : idxs) {
            if (i >= wipeCurrPos && i < wipeCurrPos + LENGTH) {
                ledBuffer.setLED(i, c1);
            } else {
                ledBuffer.setLED(i, c2);
            }
        }

        wipeCurrPos += 1;
        if (wipeCurrPos == LENGTH) {
            wipeCurrPos = -LENGTH;
        }
    }

    private void sevenColorHorseRace(int[] idxs) {
        for (int i : idxs) {
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

        if (topHalfState == bottomHalfState) 
        {
            switch(topHalfState) {
                case RAINBOW:
                    rainbow(ALL_IDXS);
                    break;
                case RED:
                    solidColor(ALL_IDXS, Color.kDarkRed);
                    break;
                case BLUE:
                    solidColor(ALL_IDXS, Color.kRoyalBlue);
                    break;
                case GREEN:
                    solidColor(ALL_IDXS, Color.kSpringGreen);
                    break;
                case ORANGE:
                    solidColor(ALL_IDXS, Color.kDarkOrange);
                case SCANNER:
                    scanner(ALL_IDXS);
                    break;
                case CHASE:
                    chase(ALL_IDXS, Color.kBlue, Color.kWhite);
                    break;
                case COLORWIPE_GREEN:
                    colorWipe(ALL_IDXS, Color.kGreen);
                    break;
                case HORSE_RACE:
                    sevenColorHorseRace(ALL_IDXS);
                    break;
                case RED_ALLIANCE:
                    chase(ALL_IDXS, Color.kFirstRed, Color.kWhite);
                    break;
                case BLUE_ALLIANCE:
                    chase(ALL_IDXS, Color.kFirstBlue, Color.kWhite);
                    break;
            }
        }
        else
        {
            switch(topHalfState) {
                case RAINBOW:
                    rainbow(TOP_IDXS);
                    break;
                case RED:
                    solidColor(TOP_IDXS, Color.kDarkRed);
                    break;
                case BLUE:
                    solidColor(TOP_IDXS, Color.kRoyalBlue);
                    break;
                case GREEN:
                    solidColor(TOP_IDXS, Color.kSpringGreen);
                    break;
                case ORANGE:
                    solidColor(TOP_IDXS, Color.kDarkOrange);
                case SCANNER:
                    scanner(TOP_IDXS);
                    break;
                case CHASE:
                    chase(TOP_IDXS, Color.kBlue, Color.kWhite);
                    break;
                case COLORWIPE_GREEN:
                    colorWipe(TOP_IDXS, Color.kGreen);
                    break;
                case HORSE_RACE:
                    sevenColorHorseRace(TOP_IDXS);
                    break;
                case RED_ALLIANCE:
                    chase(TOP_IDXS, Color.kFirstRed, Color.kWhite);
                    break;
                case BLUE_ALLIANCE:
                    chase(TOP_IDXS, Color.kFirstBlue, Color.kWhite);
                    break;
            }

            switch(bottomHalfState) {
                case RAINBOW:
                    rainbow(BOTTOM_IDXS);
                    break;
                case RED:
                    solidColor(BOTTOM_IDXS, Color.kDarkRed);
                    break;
                case BLUE:
                    solidColor(BOTTOM_IDXS, Color.kRoyalBlue);
                    break;
                case GREEN:
                    solidColor(BOTTOM_IDXS, Color.kSpringGreen);
                    break;
                case ORANGE:
                    solidColor(BOTTOM_IDXS, Color.kDarkOrange);
                case SCANNER:
                    scanner(BOTTOM_IDXS);
                    break;
                case CHASE:
                    chase(BOTTOM_IDXS, Color.kBlue, Color.kWhite);
                    break;
                case COLORWIPE_GREEN:
                    colorWipe(BOTTOM_IDXS, Color.kGreen);
                    break;
                case HORSE_RACE:
                    sevenColorHorseRace(BOTTOM_IDXS);
                    break;
                case RED_ALLIANCE:
                    chase(BOTTOM_IDXS, Color.kFirstRed, Color.kWhite);
                    break;
                case BLUE_ALLIANCE:
                    chase(BOTTOM_IDXS, Color.kFirstBlue, Color.kWhite);
                    break;
            }
        }

        

        ledStrip.setData(ledBuffer);
    }

    /**
     * Returns a color in between c1 and c2 using RGB interpolation.
     * @param c1 Color to start at (t=0)
     * @param c2 Color to end at (t=1)
     * @param t How far between the values to interpolate, clamped at [0, 1]
     * @return
     */
    private Color cInterp(Color c1, Color c2, double t) {
        t = MathUtil.clamp(t, 0, 1);

        return new Color(
            MathUtil.interpolate(c1.red, c2.red, t), 
            MathUtil.interpolate(c1.green, c2.green, t), 
            MathUtil.interpolate(c1.blue, c2.blue, t));
    }
}