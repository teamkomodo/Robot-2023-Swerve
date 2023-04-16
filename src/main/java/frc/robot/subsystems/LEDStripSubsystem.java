package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class LEDStripSubsystem extends SubsystemBase {

    public static final double IDLE_PATTERN = 0.43; //Sparkle, Color 1 on Color 2
    public static final double CONE_SIGNAL_PATTERN = 0.65; //Solid Color: Orange
    public static final double CUBE_SIGNAL_PATTERN = 0.91; //Solid Color: Violet

    private Spark controller = new Spark(LED_STRIP_PWM_CHANNEL);

    public void setPattern(double pattern) {
        controller.set(pattern);
    }

    public Command setPatternCommand(double pattern) {
        return this.runOnce(() -> setPattern(pattern));
    }

    public Command idlePatternCommand() {
        return setPatternCommand(IDLE_PATTERN);
    }
    
    public Command setSolidColor(int colorId) {
        // Not including black
        return this.runOnce(() -> setPattern(0.57 + (0.02 * (colorId % 21))));
    }

    public Command coneSignalCommand() {
        return setPatternCommand(CONE_SIGNAL_PATTERN);
    }

    public Command cubeSignalCommand() {
        return setPatternCommand(CUBE_SIGNAL_PATTERN);
    }

    public class Patterns {
        public static final double FIXED_PALETTE_PATTERN_RAINBOW_RAINBOW_PALETTE = -0.99;
        public static final double FIXED_PALETTE_PATTERN_RAINBOW_PARTY_PALETTE = -0.97;
        public static final double FIXED_PALETTE_PATTERN_RAINBOW_OCEAN_PALETTE = -0.95;
        public static final double FIXED_PALETTE_PATTERN_RAINBOW_LAVE_PALETTE = -0.93;
        public static final double FIXED_PALETTE_PATTERN_RAINBOW_FOREST_PALETTE = -0.91;
        public static final double FIXED_PALETTE_PATTERN_RAINBOW_WITH_GLITTER = -0.89;
        public static final double FIXED_PALETTE_PATTERN_CONFETTI = -0.87;
        public static final double FIXED_PALETTE_PATTERN_SHOT_RED = -0.85;
        public static final double FIXED_PALETTE_PATTERN_SHOT_BLUE = -0.83;
        public static final double FIXED_PALETTE_PATTERN_SHOT_WHITE = -0.81;
        public static final double FIXED_PALETTE_PATTERN_SINELON_RAINBOW_PALETTE = -0.79;
        public static final double FIXED_PALETTE_PATTERN_SINELON_PARTY_PALETTE = -0.77;
        public static final double FIXED_PALETTE_PATTERN_SINELON_OCEAN_PALETTE = -0.75;
        public static final double FIXED_PALETTE_PATTERN_SINELON_LAVA_PALETTE = -0.73;
        public static final double FIXED_PALETTE_PATTERN_SINELON_FOREST_PALETTE = -0.71;
        public static final double FIXED_PALETTE_PATTERN_BEATS_PER_MINUTE_RAINBOW_PALETTE = -0.69;
        public static final double FIXED_PALETTE_PATTERN_BEATS_PER_MINUTE_PARTY_PALETTE = -0.67;
        public static final double FIXED_PALETTE_PATTERN_BEATS_PER_MINUTE_OCEAN_PALETTE = -0.65;
        public static final double FIXED_PALETTE_PATTERN_BEATS_PER_MINUTE_LAVA_PALETTE = -0.63;
        public static final double FIXED_PALETTE_PATTERN_BEATS_PER_MINUTE_FOREST_PALETTE = -0.61;
        public static final double FIXED_PALETTE_PATTERN_FIRE_MEDIUM = -0.59;
        public static final double FIXED_PALETTE_PATTERN_FIRE_LARGE = -0.57;
        public static final double FIXED_PALETTE_PATTERN_TWINKLES_RAINBOW_PALETTE = -0.55;
        public static final double FIXED_PALETTE_PATTERN_TWINKLES_PARTY_PALETTE = -0.53;
        public static final double FIXED_PALETTE_PATTERN_TWINKLES_OCEAN_PALETTE = -0.51;
        public static final double FIXED_PALETTE_PATTERN_TWINKLES_LAVA_PALETTE = -0.49;
        public static final double FIXED_PALETTE_PATTERN_TWINKLES_FOREST_PALETTE = -0.47;
        public static final double FIXED_PALETTE_PATTERN_COLOR_WAVES_RAINBOW_PALETTE = -0.45;
        public static final double FIXED_PALETTE_PATTERN_COLOR_WAVES_PARTY_PALETTE = -0.43;
        public static final double FIXED_PALETTE_PATTERN_COLOR_WAVES_OCEAN_PALETTE = -0.41;
        public static final double FIXED_PALETTE_PATTERN_COLOR_WAVES_LAVA_PALETTE = -0.39;
        public static final double FIXED_PALETTE_PATTERN_COLOR_WAVES_FOREST_PALETTE = -0.37;
        public static final double FIXED_PALETTE_PATTERN_LARSON_SCANNER_RED = -0.35;
        public static final double FIXED_PALETTE_PATTERN_LARSON_SCANNER_GRAY = -0.33;
        public static final double FIXED_PALETTE_PATTERN_LIGHT_CHASE_RED = -0.31;
        public static final double FIXED_PALETTE_PATTERN_LIGHT_CHASE_BLUE = -0.29;
        public static final double FIXED_PALETTE_PATTERN_LIGHT_CHASE_GRAY = -0.27;
        public static final double FIXED_PALETTE_PATTERN_HEARTBEAT_RED = -0.25;
        public static final double FIXED_PALETTE_PATTERN_HEARTBEAT_BLUE = -0.23;
        public static final double FIXED_PALETTE_PATTERN_HEARTBEAT_WHITE = -0.21;
        public static final double FIXED_PALETTE_PATTERN_HEARTBEAT_GRAY = -0.19;
        public static final double FIXED_PALETTE_PATTERN_BREATH_RED = -0.17;
        public static final double FIXED_PALETTE_PATTERN_BREATH_BLUE = -0.15;
        public static final double FIXED_PALETTE_PATTERN_BREATH_GRAY = -0.13;
        public static final double FIXED_PALETTE_PATTERN_STROBE_RED = -0.11;
        public static final double FIXED_PALETTE_PATTERN_STROBE_BLUE = -0.09;
        public static final double FIXED_PALETTE_PATTERN_STROBE_GOLD = -0.07;
        public static final double FIXED_PALETTE_PATTERN_STROBE_WHITE = -0.05;
        public static final double COLOR_1_PATTERN_END_TO_END_BLEND_TO_BLACK = -0.03;
        public static final double COLOR_1_PATTERN_LARSON_SCANNER = -0.01;
        public static final double COLOR_1_PATTERN_LIGHT_CHASE = 0.01;
        public static final double COLOR_1_PATTERN_HEARTBEAT_SLOW = 0.03;
        public static final double COLOR_1_PATTERN_HEARTBEAT_MEDIUM = 0.05;
        public static final double COLOR_1_PATTERN_HEARTBEAT_FAST = 0.07;
        public static final double COLOR_1_PATTERN_BREATH_SLOW = 0.09;
        public static final double COLOR_1_PATTERN_BREATH_FAST = 0.11;
        public static final double COLOR_1_PATTERN_SHOT = 0.13;
        public static final double COLOR_1_PATTERN_STROBE = 0.15;
        public static final double COLOR_2_PATTERN_END_TO_END_BLEND_TO_BLACK = 0.17;
        public static final double COLOR_2_PATTERN_LARSON_SCANNER = 0.19;
        public static final double COLOR_2_PATTERN_LIGHT_CHASE = 0.21;
        public static final double COLOR_2_PATTERN_HEARTBEAT_SLOW = 0.23;
        public static final double COLOR_2_PATTERN_HEARTBEAT_MEDIUM = 0.25;
        public static final double COLOR_2_PATTERN_HEARTBEAT_FAST = 0.27;
        public static final double COLOR_2_PATTERN_BREATH_SLOW = 0.29;
        public static final double COLOR_2_PATTERN_BREATH_FAST = 0.31;
        public static final double COLOR_2_PATTERN_SHOT = 0.33;
        public static final double COLOR_2_PATTERN_STROBE = 0.35;
        public static final double COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_1_ON_COLOR_2 = 0.37;
        public static final double COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1 = 0.39;
        public static final double COLOR_1_AND_2_PATTERN_COLOR_GRADIENT_COLOR_1_AND_2 = 0.41;
        public static final double COLOR_1_AND_2_PATTERN_BEATS_PER_MINUTE_COLOR_1_AND_2 = 0.43;
        public static final double COLOR_1_AND_2_PATTERN_END_TO_END_BLEND_COLOR_1_TO_2 = 0.45;
        public static final double COLOR_1_AND_2_PATTERN_END_TO_END_BLEND = 0.47;
        public static final double COLOR_1_AND_2_PATTERN_COLOR_1_AND_COLOR_2_NO_BLENDING = 0.49;
        public static final double COLOR_1_AND_2_PATTERN_TWINKLES_COLOR_1_AND_2 = 0.51;
        public static final double COLOR_1_AND_2_PATTERN_COLOR_WAVES_COLOR_1_AND_2 = 0.53;
        public static final double COLOR_1_AND_2_PATTERN_SINELON_COLOR_1_AND_2 = 0.55;
        public static final double SOLID_COLORS_HOT_PINK = 0.57;
        public static final double SOLID_COLORS_DARK_RED = 0.59;
        public static final double SOLID_COLORS_RED = 0.61;
        public static final double SOLID_COLORS_RED_ORANGE = 0.63;
        public static final double SOLID_COLORS_ORANGE = 0.65;
        public static final double SOLID_COLORS_GOLD = 0.67;
        public static final double SOLID_COLORS_YELLOW = 0.69;
        public static final double SOLID_COLORS_LAWN_GREEN = 0.71;
        public static final double SOLID_COLORS_LIME = 0.73;
        public static final double SOLID_COLORS_DARK_GREEN = 0.75;
        public static final double SOLID_COLORS_GREEN = 0.77;
        public static final double SOLID_COLORS_BLUE_GREEN = 0.79;
        public static final double SOLID_COLORS_AQUA = 0.81;
        public static final double SOLID_COLORS_SKY_BLUE = 0.83;
        public static final double SOLID_COLORS_DARK_BLUE = 0.85;
        public static final double SOLID_COLORS_BLUE = 0.87;
        public static final double SOLID_COLORS_BLUE_VIOLET = 0.89;
        public static final double SOLID_COLORS_VIOLET = 0.91;
        public static final double SOLID_COLORS_WHITE = 0.93;
        public static final double SOLID_COLORS_GRAY = 0.95;
        public static final double SOLID_COLORS_DARK_GRAY = 0.97;
        public static final double SOLID_COLORS_BLACK = 0.99;
    }
}
