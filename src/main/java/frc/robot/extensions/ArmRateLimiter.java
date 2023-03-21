package frc.robot.extensions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * A class that limits the rate of change of an input value with consideration for gravity. Based on the SlewRateLimiter {@link
 * edu.wpi.first.math.filter.SlewRateLimiter}.
 */
public class ArmRateLimiter {

  private final double m_positiveRateLimit;
  private final double m_negativeRateLimit;
  private final double m_angleAtHorizontal;
  private final double m_gravityUp;
  private final double m_gravityDown;
  private double m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new ArmRateLimiter that compensates for effects of gravity. Angles must increase as mechanism rises from horizontal to vertical. 
   *
   * @param positiveRateLimit The rate-of-change limit in the positive direction, in degrees per
   *     second. This is expected to be positive.
   * @param negativeRateLimit The rate-of-change limit in the negative direction, in degrees per
   *     second. This is expected to be negative.
   * @param initialValue The initial angle of the mechanism in degrees.
   * @param angleAtHorizontal Reported angle when mechanism is level with ground. 
   * @param gravityUpwardAssist Amount to assist mechanism as it moves up. (A value of 1.0 will double the rate-of-change at horizontal).
   * @param gravityDownwardAssist Amount to assist mechanism as it moves down. (A value of 1.0 would zero the rate-of-change at horizontal).
   */
  public ArmRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue, double angleAtHorizontal, double gravityUpwardAssist, double gravityDownwardAssist) {
    m_positiveRateLimit = positiveRateLimit;
    m_negativeRateLimit = negativeRateLimit;
    m_angleAtHorizontal = angleAtHorizontal;
    m_gravityUp = gravityUpwardAssist;
    m_gravityDown = gravityDownwardAssist;

    m_prevVal = initialValue;
    m_prevTime = getTimestamp();
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param targetAngle The input value whose slew rate is to be limited.
   * @param currentAngle The angle in degrees.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double targetAngle, double currentAngle) {
    double currentTime = getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    //calculate the cosine by first converting to radians
    double radianCosine = Math.cos((currentAngle - m_angleAtHorizontal) * Math.PI / 180.0);
    m_prevVal +=
        MathUtil.clamp(
            targetAngle - m_prevVal,
            m_negativeRateLimit * elapsedTime * (1 - this.m_gravityDown * radianCosine), // slower as it drops to horizontal
            m_positiveRateLimit * elapsedTime * ( 1 + this.m_gravityUp * radianCosine)  // faster when rising from the ground
        );
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = getTimestamp();
  }

  public static double getTimestamp() {
    return WPIUtilJNI.now() * 1.0e-6;
  }

}
