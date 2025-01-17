package MidnightLibrary.MidnightMovement;

/**
 * Created by Archish on 5/5/18.
 */

public enum MidnightMotorModel {
    ORBITAL20, NEVEREST40, NEVEREST60, USDIGITAL_E4T, REVHDHEX40, NEVERREST_CLASSIC, NEVERREST256, REVHDHEX20, REVTHROUGHBORE, NEVERREST37, REVHDHEX1;
    public static double DEFAULT_CPR = 537.6;
    public static int DEFAULT_RPM = 150;

    public static double CPR(MidnightMotorModel motorModel) {
        switch (motorModel) {
            case ORBITAL20:
                return 537.6;
            case REVHDHEX20:
                return 560;
            case NEVEREST40:
            case REVHDHEX40:
                return 1120;
            case NEVEREST60:
                return 1680;
            case USDIGITAL_E4T:
                return 1440;
            case NEVERREST_CLASSIC:
                return 28;
            case NEVERREST256:
                return 4400;
            case REVTHROUGHBORE:
                return 8192;
            case NEVERREST37:
                return 44.4;
            case REVHDHEX1:
                return 38;
        }
        return DEFAULT_CPR;
    }

    public static int RPM(MidnightMotorModel motorModel) {
        switch (motorModel) {
            case ORBITAL20:
                return 315;
            case NEVEREST40:
                return 160;
            case NEVEREST60:
                return 105;
            case REVHDHEX40:
                return 150;
            case NEVERREST_CLASSIC:
                return 6600;
            case REVHDHEX20:
                return 300;
            case NEVERREST37:
                return 1780;
            case REVHDHEX1:
                return 6000;
        }
        return DEFAULT_RPM;
    }

    public double CPR() {
        return CPR(this);
    }

    public int RPM() {
        return RPM(this);
    }
}