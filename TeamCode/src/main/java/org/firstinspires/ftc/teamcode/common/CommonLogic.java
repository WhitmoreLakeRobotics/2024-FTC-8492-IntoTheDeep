package org.firstinspires.ftc.teamcode.common;

public class CommonLogic extends Object {

    //*********************************************************************************************
    public static double CapValue(double TargetValue, double negCapValue, double posCapValue) {
        // logic to cap the motor power between a good range
        double retValue = TargetValue;

        if (TargetValue < negCapValue) {
            retValue = negCapValue;
        }

        if (TargetValue > posCapValue) {
            retValue = posCapValue;
        }

        return retValue;
    }
    public static int CapValueint(int TargetValue, int negCapValue, int posCapValue) {
        // logic to cap the motor power between a good range
        int retValue = TargetValue;

        if (TargetValue < negCapValue) {
            retValue = negCapValue;
        }

        if (TargetValue > posCapValue) {
            retValue = posCapValue;
        }

        return retValue;
    }
    //*********************************************************************************************
    public static double joyStickMath(double joyValue) {
        int sign = 1;
        double retValue = 0;
        if (joyValue < 0) {
            sign = -1;
        }
        return Math.abs(Math.pow(joyValue, 2)) * sign;
    }

    //*********************************************************************************************
    public static boolean inRange(double value, double targetValue, double tol) {
        // function to tell if an encoder is within tolerance

        boolean retValue = false;

        if ((value >= (targetValue - tol)) && (value <= (targetValue + tol))) {
            retValue = true;
        }

        return retValue;

    }
    //*********************************************************************************************

    public static boolean indexCheck(int value, int low, int high) {

        return (value >= low && value <= high);
    }

    // returns true only on the previous state was false and the current state is true
    // AKA a button is freshly pressed do something.   If it as already been pressed
    // it is not worth doing again.
    public static boolean oneShot(boolean currState, boolean prevState) {

        return (prevState == false && currState == true);
    }

    //*********************************************************************************************

    public static double goToPosition(int CurrentPos, int TargetPos, int Tol, double NegPower, double PosPower, double HoldPower, int stag) {
       double Power = HoldPower;
        //If current pos greater than target pos then add negative power,
        if (CurrentPos > (TargetPos + stag)) {
            Power = NegPower;
            //else if current pos less than target pos add positive power,
        } else if (CurrentPos < (TargetPos - stag)) {
            Power = PosPower;
        } else if (CurrentPos > (TargetPos + Tol)) {
            Power = NegPower * .65;
            //else if current pos less than target pos add positive power,
        } else if (CurrentPos < (TargetPos - Tol)) {
            Power = PosPower * .65;
        }
        else {
            Power = 1/HoldPower;
        }

        //else if current pos (in tolerance) stop/hold
        return Power;
    }

    public static double goToPosStag(double currentPos, double targetPos, double tol, double power, double stagStart, double stagPower){


        //if in range stop
        if(inRange(currentPos, targetPos, tol)){
            return 0;
        }
        else if(currentPos<=(targetPos+stagStart)&&currentPos>targetPos){
            return -stagPower;
        }
        else if(currentPos>=(targetPos-stagStart)&&currentPos<targetPos){
            return stagPower;
        }
        else if((currentPos < targetPos)) {
            return power;
        }
        //if after pos. subract power
        else if(currentPos > targetPos) {
            return -1*power;
        }
        else {
            return 0;
        }
    }

    public static double goToPosStagint(int currentPos, int targetPos, int tol, double power, double stagStart, double stagPower){


        //if in range stop
        if(inRange(currentPos, targetPos, tol)){
            return 0;
        }
        else if(currentPos<=(targetPos+stagStart)&&currentPos>targetPos){
            return -stagPower;
        }
        else if(currentPos>=(targetPos-stagStart)&&currentPos<targetPos){
            return stagPower;
        }
        else if((currentPos < targetPos)) {
            return power;
        }
        //if after pos. subract power
        else if(currentPos > targetPos) {
            return -1*power;
        }
        else {
            return 0;
        }
    }

    public static double PIDcalc(double P, double F_hold,int currentPos, int targetPos){
        int delta = targetPos - currentPos;

        return (delta / P) + F_hold;

    }

}
