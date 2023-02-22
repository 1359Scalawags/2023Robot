package frc.robot.extensions;

import edu.wpi.first.math.Vector;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PidConfig implements Sendable {
    private double p;
    private double i;
    private double d;
    private double minP, maxP;
    private double minI, maxI;
    private double minD, maxD;
    private double tuningScale;

    private static double defaultMax = 0.0001;

    public PidConfig(double p, double i, double d, double tuningScale) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.minP = 0;
        this.maxP = defaultMax;
        this.minI = 0;
        this.maxI = defaultMax;
        this.minD = 0;
        this.maxD = defaultMax;
        this.tuningScale = tuningScale;
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public void setP(double p) {
        this.p = p;
    }

    public void setI(double i) {
        this.i = i;
    }

    public void setD(double d) {
        this.d = d;
    }
    
    public double getMaxP() {
        return maxP;
    }

    public double getMinP() {
        return minP;
    }

    public double getMaxI() {
        return maxI;
    }

    public double getMinI() {
        return minI;
    }

    public double getMaxD() {
        return maxD;
    }

    public double getMinD() {
        return minD;
    }



    private void safeState() {
        p = 0;
        i = 0;
        d = 0;
    }

    private double getTuningScale() {
        return tuningScale;
    }

    private void setTuningScale(double value) {
        tuningScale = value;
    }

    private double getScaledP() {
        return p / tuningScale;
    }

    private void setScaledP(double value) {
        p = value * tuningScale;
    }

    private double getScaledI() {
        return i / tuningScale;
    }

    private void setScaledI(double value) {
        i = value * tuningScale;
    }

    private double getScaledD() {
        return i / tuningScale;
    }

    private void setScaledD(double value) {
        i = value * tuningScale;
    }

    private void setScaledRangeP(double[] range) {
        this.minP = range[0] * tuningScale;
        this.maxP = range[1] * tuningScale;
    }

    private void setScaledRangeI(double[] range) {
        this.minI = range[0] * tuningScale;
        this.maxI = range[1] * tuningScale;
    }

    private void setScaledRangeD(double[] range) {
        this.minD = range[0] * tuningScale;
        this.maxD = range[1] * tuningScale;
    }

    private double[] getScaledRangeP() {
        return new double[] {minP / tuningScale, maxP / tuningScale};
    }
    private double[] getScaledRangeI() {
        return new double[] {minI / tuningScale, maxI / tuningScale};
    }
    private double[] getScaledRangeD() {
        return new double[] {minD / tuningScale, maxD / tuningScale};
    }    


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::safeState);
        builder.addDoubleProperty("Multiplier", this::getTuningScale, this::setTuningScale);
        builder.addDoubleProperty("P", this::getScaledP, this::setScaledP);
        builder.addDoubleProperty("i", this::getScaledI, this::setScaledI);
        builder.addDoubleProperty("d", this::getScaledD, this::setScaledD);
        builder.addDoubleArrayProperty("P Range", this::getScaledRangeP, this::setScaledRangeP);
        builder.addDoubleArrayProperty("I Range", this::getScaledRangeI, this::setScaledRangeI);        
        builder.addDoubleArrayProperty("D Range", this::getScaledRangeD, this::setScaledRangeD);
        
    }





}
