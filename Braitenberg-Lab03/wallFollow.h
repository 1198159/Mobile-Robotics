private final float kPa = 1;
private final float kIa = 0;
private final float kDa = 0; 

private final float maxLinSpeed = 100; //mm/s
private final float maxAngSpeed = 1; //rad/s
private final float targetDistance = 127; //mm

private final float maxAngSpeedIntegral = .1;

private float angSpeedIntegral = 0;
private float errorDistLast = NaN;