package org.firstinspires.ftc.teamcode.teleOp.subSystems;

public enum Mosaic {
    PPG,
    GPP,
    PGP,
    UNKNOWN;

    /**
     *
     * @param id April Tag fiducial ID
     * @return Mosaic pattern
     * @author Harry Wiencek
     */
    public static Mosaic fromId(byte id) {
        switch (id) {
            case 21:
                return GPP;
            case 22:
                return PPG;
            case 23:
                return PGP;
            default:
                return UNKNOWN;
        }
    }
}
