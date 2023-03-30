package simulation;
public class PID {


    private double P;
    private double I;
    private double D;
    private double lastError;
    private boolean firstRun;

    private double integral;
    public PID(double p, double i, double d) {
        this.P = p;
        this.I= i;
        this.D = d;
        this.firstRun=true;
        this.integral=0;
    }

    public double update(double error, double dt) {
        if (this.firstRun) {
            this.firstRun = false;
            this.lastError = error;
        }
        double diff = (error - this.lastError) / dt;
        integral+=error*dt;
        double controlOut = P*error + I*integral + D*diff;
        this.lastError = error;
        return controlOut;
    }


}