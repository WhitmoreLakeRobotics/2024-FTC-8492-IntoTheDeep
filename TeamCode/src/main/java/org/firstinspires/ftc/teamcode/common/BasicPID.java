package org.firstinspires.ftc.teamcode.common;


/*
PID Functions
True to the name, the main purpose of this class is PID control.

Feed Forward rate setting.
Provides an "Expected output", helpful when doing velocity control systems.

Setpoint Range
Force the PID system to cap the setpoint to a range near the current input values. This allows you to tune the PID in a smaller range, and have it perform sanely during large setpoint changes.

Output ramping
Allows a maximum rate of change on the output, preventing jumps in output during setpoint changes

Output Range
Adjustable min and maximum range, so the output can directly drive a variety of systems.

Output Filtering
Helps smooth the output, preventing high-frequency oscillations.

I term restriction
Allows you to specify a maximum output value the I term will generate. This allows active error correction,
with minimal risk of I-term windup.

Smart I term buildup
The I term and summed error will never increase if the system is already doing everything permitted to correct the system error.

Simple API.
No need for lots of convoluted calculation functions, or asynchronous calculation modes.
After configuration, getOutput() is probably the only function you need.

Usage
A bare bones PID system could look like this.

******************************************************************
        BasicPID pid=BasicPID(1,0,0);
        //set any other PID configuration options here.

        while(true){
          //get some sort of sensor value
          //set some sort of target value
          double output=pid.getOutput(sensor,target);
          //do something with the output
          Timer.delay(50); // Wait for the next loop
        }
******************************************************************

 */

public class BasicPID {

    //**********************************
    // Class private variables
    //**********************************

    private double KP=0;
    private double KI=0;
    private double KD=0;
    private double KF=0;

    private double maxIntegralOutput=0;
    private double maxError=0;
    private double errorSum=0;

    private double maxOutput=0;
    private double minOutput=0;

    private double setpoint=0;

    private double lastActual=0;

    private boolean firstRun=true;
    private boolean reversed=false;

    private double outputRampRate=0;
    private double lastOutput=0;

    private double outputFilter=0;

    private double setpointRange=0;

    //**********************************
    // Constructor Methods
    //**********************************

    /**
     * Create a PID class object.
     * See setP, setI, setD methods for more detailed parameters.
     * @param kp Proportional gain. Large if large difference between setpoint and target.
     * @param ki Integral gain.  Becomes large if setpoint cannot reach target quickly.
     * @param kd Derivative gain. Responds quickly to large changes in error. Small values prevents KP and KI terms from causing overshoot.
     */
    public BasicPID(double kp, double ki, double kd){
        KP=kp; KI=ki; KD=kd;
        checkSigns();
    }

    /**
     * Create a PID class object.
     * See setP, setI, setD, setF methods for more detailed parameters.
     * @param kp Proportional gain. Large if large difference between setpoint and target.
     * @param ki Integral gain.  Becomes large if setpoint cannot reach target quickly.
     * @param kd Derivative gain. Responds quickly to large changes in error. Small values prevents KP and KI terms from causing overshoot.
     * @param kf Feed-forward gain. Open loop "best guess" for the output should be. Only useful if setpoint represents a rate.
     */
    public BasicPID(double kp, double ki, double kd, double kf){
        KP=kp; KI=ki; KD=kd; KF=kf;
        checkSigns();
    }

    //**********************************
    // Configuration Methods
    //**********************************
    /**
     * Configure the Proportional gain parameter. <br>
     * This responds quickly to changes in setpoint, and provides most of the initial driving force
     * to make corrections. <br>
     * Some systems can be used with only a KP gain, and many can be operated with only PI.<br>
     * For position based controllers, this is the first parameter to tune, with KI second. <br>
     * For rate controlled systems, this is often the second after KF.
     *
     * @param kp Proportional gain. Affects output according to <b>output+=KP*(setpoint-current_value)</b>
     */
    public void setP(double kp){
        KP=kp;
        checkSigns();
    }

    /**
     * Changes the KI parameter <br>
     * This is used for overcoming disturbances, and ensuring that the controller always gets to the control mode. 
     * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. <br>
     * Affects output through <b>output+=previous_errors*Igain ;previous_errors+=current_error</b>
     *
     * @see {@link #setmaxIntegralOutput(double) setmaxIntegralOutput} for how to restrict
     *
     * @param ki New gain value for the Integral term
     */
    public void setI(double ki){
        if(KI!=0){
            errorSum=errorSum*KI/ki;
        }
        if(maxIntegralOutput!=0){
            maxError=maxIntegralOutput/ki;
        }
        KI=ki;
        checkSigns();
        // Implementation note: 
        // This Scales the accumulated error to avoid output errors. 
        // As an example doubling the KI term cuts the accumulated error in half, which results in the 
        // output change due to the KI term constant during the transition. 
    }

    /**
     * Changes the KD parameter <br>
     * This has two primary effects:
     * <list>
     * <li> Adds a "startup kick" and speeds up system response during setpoint changes
     * <li> Adds "drag" and slows the system when moving toward the target
     * </list>
     * A small KD value can be useful for both improving response times, and preventing overshoot.
     * However, in many systems a large KD value will cause significant instability, particularly 
     * for large setpoint changes.
     * <br>
     * Affects output through <b>output += -KD*(current_input_value - last_input_value)</b>
     *
     * @param kd New gain value for the Derivative term
     */
    public void setD(double kd){
        KD=kd;
        checkSigns();
    }

    /**
     * Configure the FeedForward parameter. <br>
     * This is excellent for velocity, rate, and other  continuous control modes where you can 
     * expect a rough output value based solely on the setpoint.<br>
     * Should not be used in "position" based control modes.<br>
     * Affects output according to <b>output+=KF*Setpoint</b>. Note, that a KF-only system is actually open loop.
     *
     * @param kf Feed forward gain.
     */
    public void setF(double kf){
        KF=kf;
        checkSigns();
    }

    /**
     * Configure the PID object.
     * See setP, setI, setD methods for more detailed parameters.
     * @param kp Proportional gain. Large if large difference between setpoint and target.
     * @param ki Integral gain.  Becomes large if setpoint cannot reach target quickly.
     * @param kd Derivative gain. Responds quickly to large changes in error. Small values prevents KP and KI terms from causing overshoot.
     */
    public void setPID(double kp, double ki, double kd){
        KP=kp;KD=kd;
        //Note: the KI term has additional calculations, so we need to use it's 
        //specific method for setting it.
        setI(ki);
        checkSigns();
    }

    /**
     * Configure the PID object.
     * See setP, setI, setD, setF methods for more detailed parameters.
     * @param kp Proportional gain. Large if large difference between setpoint and target.
     * @param ki Integral gain.  Becomes large if setpoint cannot reach target quickly.
     * @param kd Derivative gain. Responds quickly to large changes in error. Small values prevents KP and KI terms from causing overshoot.
     * @param kf Feed-forward gain. Open loop "best guess" for the output should be. Only useful if setpoint represents a rate.
     */
    public void setPID(double kp, double ki, double kd,double kf){
        KP=kp;KD=kd;KF=kf;
        //Note: the KI term has additional calculations, so we need to use it's 
        //specific method for setting it.
        setI(ki);
        checkSigns();
    }

    /**
     * Set the maximum output value contributed by the KI component of the system
     * This can be used to prevent large windup issues and make tuning simpler
     * @param maximum Units are the same as the expected output value
     */
    public void setmaxIntegralOutput(double maximum){
        // Internally maxError and Izone are similar, but scaled for different purposes. 
        // The maxError is generated for simplifying math, since calculations against 
        // the max error are far more common than changing the KI term or Izone. 
        maxIntegralOutput=maximum;
        if(KI!=0){
            maxError=maxIntegralOutput/KI;
        }
    }

    /**
     * Specify a maximum output range. <br>
     * When one input is specified, output range is configured to 
     * <b>[-output, output]</b>
     * @param output
     */
    public void setOutputLimits(double output){
        setOutputLimits(-output,output);
    }

    /**
     * Specify a  maximum output.
     * When two inputs specified, output range is configured to 
     * <b>[minimum, maximum]</b>
     * @param minimum possible output value
     * @param maximum possible output value
     */
    public void setOutputLimits(double minimum,double maximum){
        if(maximum<minimum)return;
        maxOutput=maximum;
        minOutput=minimum;

        // Ensure the bounds of the KI term are within the bounds of the allowable output swing
        if(maxIntegralOutput==0 || maxIntegralOutput>(maximum-minimum) ){
            setmaxIntegralOutput(maximum-minimum);
        }
    }

    /**
     * Set the operating direction of the PID controller
     * @param reversed Set true to reverse PID output
     */
    public void  setDirection(boolean reversed){
        this.reversed=reversed;
    }

    //**********************************
    // Primary operating functions
    //**********************************

    /**
     * Configure setpoint for the PID calculations<br>
     * This represents the target for the PID system's, such as a 
     * position, velocity, or angle. <br>
     * @see BasicPID#getOutput(actual) <br>
     * @param setpoint
     */
    public void setSetpoint(double setpoint){
        this.setpoint=setpoint;
    }

    /**
     * Calculate the output value for the current PID cycle.<br>
     * @param actual The monitored value, typically as a sensor input.
     * @param setpoint The target value for the system
     * @return calculated output value for driving the system
     */
    public double getOutput(double actual, double setpoint){
        double output;
        double Poutput;
        double Ioutput;
        double Doutput;
        double Foutput;

        this.setpoint=setpoint;

        // Ramp the setpoint used for calculations if user has opted to do so
        if(setpointRange!=0){
            setpoint=constrain(setpoint,actual-setpointRange,actual+setpointRange);
        }

        // Do the simple parts of the calculations
        double error=setpoint-actual;

        // Calculate KF output. Notice, this depends only on the setpoint, and not the error. 
        Foutput=KF*setpoint;

        // Calculate KP term
        Poutput=KP*error;

        // If this is our first time running this, we don't actually _have_ a previous input or output. 
        // For sensor, sanely assume it was exactly where it is now.
        // For last output, we can assume it's the current time-independent outputs. 
        if(firstRun){
            lastActual=actual;
            lastOutput=Poutput+Foutput;
            firstRun=false;
        }

        // Calculate KD Term
        // Note, this is negative. This actually "slows" the system if it's doing
        // the correct thing, and small values helps prevent output spikes and overshoot 
        Doutput= -KD*(actual-lastActual);
        lastActual=actual;

        // The Iterm is more complex. There's several things to factor in to make it easier to deal with.
        // 1. maxIntegralOutput restricts the amount of output contributed by the Iterm.
        // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
        // 3. prevent windup by not increasing errorSum if output is output=maxOutput    
        Ioutput=KI*errorSum;
        if(maxIntegralOutput!=0){
            Ioutput=constrain(Ioutput,-maxIntegralOutput,maxIntegralOutput);
        }

        // And, finally, we can just add the terms up
        output=Foutput + Poutput + Ioutput + Doutput;

        // Figure out what we're doing with the error.
        if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
            errorSum=error;
            // reset the error sum to a sane level
            // Setting to current error ensures a smooth transition when the KP term 
            // decreases enough for the KI term to start acting upon the controller
            // From that point the KI term will build up as would be expected
        }
        else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
            errorSum=error;
        }
        else if(maxIntegralOutput!=0){
            errorSum=constrain(errorSum+error,-maxError,maxError);
            // In addition to output limiting directly, we also want to prevent KI term 
            // buildup, so restrict the error directly
        }
        else{
            errorSum+=error;
        }

        // Restrict output to our specified output and ramp limits
        if(outputRampRate!=0){
            output=constrain(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
        }
        if(minOutput!=maxOutput){
            output=constrain(output, minOutput,maxOutput);
        }
        if(outputFilter!=0){
            output=lastOutput*outputFilter+output*(1-outputFilter);
        }

        // Get a test printline with lots of details about the internal 
        // calculations. This can be useful for debugging. 
        // System.out.printf("Final output %5.2f [ %5.2f, %5.2f , %5.2f  ], eSum %.2f\n",output,Poutput, Ioutput, Doutput,errorSum );
        // System.out.printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\n",output,Poutput, Ioutput, Doutput );

        lastOutput=output;
        return output;
    }

    /**
     * Calculate the output value for the current PID cycle.<br>
     * In no-parameter mode, this uses the last sensor value, 
     * and last setpoint value. <br>
     * Not typically useful, and use of parameter modes is suggested. <br>
     * @return calculated output value for driving the system
     */
    public double getOutput(){
        return getOutput(lastActual,setpoint);
    }

    /**
     * Calculate the output value for the current PID cycle.<br>
     * In one parameter mode, the last configured setpoint will be used.<br>
     * @see BasicPID#setSetpoint()
     * @param actual The monitored value, typically as a sensor input.
     * @return calculated output value for driving the system
     */
    public double getOutput(double actual){
        return getOutput(actual,setpoint);
    }

    /**
     * Resets the controller. This erases the KI term buildup, and removes 
     * KD gain on the next loop.<br>
     * This should be used any time the PID is disabled or inactive for extended
     * duration, and the controlled portion of the system may have changed due to
     * external forces.
     */
    public void reset(){
        firstRun=true;
        errorSum=0;
    }

    /**
     * Set the maximum rate the output can increase per cycle.<br>
     * This can prevent sharp jumps in output when changing setpoints or 
     * enabling a PID system, which might cause stress on physical or electrical
     * systems.  <br>
     * Can be very useful for fast-reacting control loops, such as ones 
     * with large KP or KD values and feed-forward systems.
     *
     * @param rate, with units being the same as the output
     */
    public void setOutputRampRate(double rate){
        outputRampRate=rate;
    }

    /**
     * Set a limit on how far the setpoint can be from the current position
     * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range. 
     * <br>This limits the reactivity of KP term, and restricts impact of large KD term
     * during large setpoint adjustments. Increases lag and KI term if range is too small.
     * @param range, with units being the same as the expected sensor range. 
     */
    public void setSetpointRange(double range){
        setpointRange=range;
    }

    /**
     * Set a filter on the output to reduce sharp oscillations. <br>
     * 0.1 is likely a sane starting value. Larger values use historical data
     * more heavily, with low values weigh newer data. 0 will disable, filtering, and use 
     * only the most recent value. <br>
     * Increasing the filter strength will KP and KD oscillations, but force larger KI 
     * values and increase KI term overshoot.<br>
     * Uses an exponential wieghted rolling sum filter, according to a simple <br>
     * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre> algorithm.
     * @param output valid between [0..1), meaning [current output only.. historical output only)
     */
    public void setOutputFilter(double strength){
        if(strength==0 || bounded(strength,0,1)){
            outputFilter=strength;
        }
    }

    //**************************************
    // Helper functions
    //**************************************

    /**
     * Forces a value into a specific range
     * @param value input value
     * @param min maximum returned value
     * @param max minimum value in range
     * @return Value if it's within provided range, min or max otherwise 
     */
    private double constrain(double value, double min, double max){
        if(value > max){ return max;}
        if(value < min){ return min;}
        return value;
    }

    /**
     * Test if the value is within the min and max, inclusive
     * @param value to test
     * @param min Minimum value of range
     * @param max Maximum value of range
     * @return true if value is within range, false otherwise
     */
    private boolean bounded(double value, double min, double max){
        // Note, this is an inclusive range. This is so tests like
        // `bounded(constrain(0,0,1),0,1)` will return false.
        // This is more helpful for determining edge-case behaviour
        // than <= is.
        return (min<value) && (value<max);
    }

    /**
     * To operate correctly, all PID parameters require the same sign
     * This should align with the {@literal}reversed value
     */
    private void checkSigns(){
        if(reversed){  // all values should be below zero
            if(KP>0) KP*=-1;
            if(KI>0) KI*=-1;
            if(KD>0) KD*=-1;
            if(KF>0) KF*=-1;
        }
        else{  // all values should be above zero
            if(KP<0) KP*=-1;
            if(KI<0) KI*=-1;
            if(KD<0) KD*=-1;
            if(KF<0) KF*=-1;
        }
    }
    
}
