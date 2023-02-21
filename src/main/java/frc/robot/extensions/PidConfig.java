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

    private static double defaultMax = 0.0001;

    public PidConfig(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.minP = 0;
        this.maxP = defaultMax;
        this.minI = 0;
        this.maxI = defaultMax;
        this.minD = 0;
        this.maxD = defaultMax;
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

    private void setRangeP(double[] range) {
        this.minP = range[0];
        this.maxP = range[1];
    }

    private void setRangeI(double[] range) {
        this.minI = range[0];
        this.maxI = range[1];
    }

    private void setRangeD(double[] range) {
        this.minD = range[0];
        this.maxD = range[1];
    }

    private double[] getRangeP() {
        return new double[] {minP, maxP};
    }
    private double[] getRangeI() {
        return new double[] {minI, maxI};
    }
    private double[] getRangeD() {
        return new double[] {minD, maxD};
    }    


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::safeState);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleArrayProperty("P Range", this::getRangeP, this::setRangeP);
        builder.addDoubleArrayProperty("I Range", this::getRangeI, this::setRangeI);        
        builder.addDoubleArrayProperty("D Range", this::getRangeD, this::setRangeD);
        
    }





}
