package org.firstinspires.ftc.teamcode.common;




/* Explanation *************************************************************************************

rateLimit: Specifies the maximum rate of change allowed, in units per second.

lastValue: Stores the previous output value.

lastTime: Tracks the timestamp of the last calculation.

calculate(): This method takes a new input value and applies the slew rate limiting logic.
It calculates the maximum allowed change based on the elapsed time and the rate limit,
then adjusts the output value accordingly.

reset(): Resets the limiter to a specified value, ignoring the rate limit for that initial reset.

 **************************************************************************************/

/* USAGE *************************************************************************************

    SlewRateLimiter limiter = new SlewRateLimiter(0.5); // Limit rate of change to 0.5 units per second

    double inputValue = 10.0;
    double limitedValue = limiter.calculate(inputValue);

    System.out.println(limitedValue);

 **************************************************************************************/
public class SlewRateLimiter {

    private double rateLimit; // Rate limit in units per second
    private double lastValue; // Last output value
    private long lastTime; // Last update time in milliseconds

    public SlewRateLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
        this.lastValue = 0.0;
        this.lastTime = System.currentTimeMillis();
    }

    public double calculate(double input) {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0; // Time delta in seconds

        double maxChange = rateLimit * deltaTime;
        double desiredChange = input - lastValue;

        double change = Math.max(Math.min(desiredChange, maxChange), -maxChange);

        lastValue += change;
        lastTime = currentTime;

        return lastValue;
    }

    public void reset(double value) {
        this.lastValue = value;
        this.lastTime = System.currentTimeMillis();
    }

}
