package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

public class SensorModelTagJacobian implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
    @Override
    public SimpleMatrix apply(SimpleMatrix mu, SimpleMatrix j_t) {
        double x = mu.get(0);
        double y = mu.get(1);
        double z = mu.get(2);

        double rw = mu.get(3);
        double rx = mu.get(4);
        double ry = mu.get(5);
        double rz = mu.get(6);

        int j = (int) j_t.get(0);
        int offset = 7 * (j + 1);
        double jx = mu.get(0 + offset);
        double jy = mu.get(1 + offset);
        double jz = mu.get(2 + offset);

        double jrw = mu.get(3 + offset);
        double jrx = mu.get(4 + offset);
        double jry = mu.get(5 + offset);
        double jrz = mu.get(6 + offset);

        SimpleMatrix H = new SimpleMatrix(7, 14);

        H.set(0, (-Math.pow(rw, 2) - Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(1, 2 * (-rw * rz - rx * ry) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(2, 2 * (rw * ry - rx * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(3, (-rw * rx * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                + rw * (rw * (rw * (-jx + x) + ry * (jz - z) - rz * (jy - y))
                        + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + rw * (rw * (-rw * (jx - x) + ry * (jz - z) - rz * (jy - y))
                        - rx * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                        + ry * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                        - rz * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x)))
                - ry * (rw * (rw * (-jz + z) + rx * (jy - y) - ry * (jx - x))
                        + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + rz * (rw * (rw * (-jy + y) - rx * (jz - z) + rz * (jx - x))
                        + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                        * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(4, (-rw * rx * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                + rx * (rx * (rx * (-jx + x) - ry * (jy - y) - rz * (jz - z))
                        + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + rx * (-rw * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                        - rx * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                        + ry * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                        - rz * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x)))
                + ry * (rx * (rw * (jz - z) + rx * (-jy + y) + ry * (jx - x))
                        + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + rz * (rx * (-rw * (jy - y) + rx * (-jz + z) + rz * (jx - x))
                        + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                        * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(5, (-rw
                * (ry * (rw * (jx - x) + ry * (-jz + z) + rz * (jy - y))
                        + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + rx * (ry * (-rx * (jx - x) + ry * (-jy + y) - rz * (jz - z))
                        + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                - ry * rz * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                - ry * (ry * (-rw * (jz - z) + rx * (jy - y) + ry * (-jx + x))
                        + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + ry * (-rw * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                        - rx * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                        + ry * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                        - rz * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x)))
                + (-rw * (jz - z) + rx * (jy - y) - ry * (jx - x))
                        * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(6, (rw
                * (rz * (-rw * (jx - x) + ry * (jz - z) + rz * (-jy + y))
                        + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + rx * (rz * (-rx * (jx - x) - ry * (jy - y) + rz * (-jz + z))
                        + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + ry * rz * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                - rz * (rz * (rw * (jy - y) + rx * (jz - z) + rz * (-jx + x))
                        + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                + rz * (-rw * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                        - rx * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                        + ry * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                        + rz * (-rw * (jy - y) - rx * (jz - z) + rz * (jx - x)))
                + (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                        * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(7, (Math.pow(rw, 2) + Math.pow(rx, 2) - Math.pow(ry, 2) - Math.pow(rz, 2))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(8, 2 * (rw * rz + rx * ry) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(9, 2 * (-rw * ry + rx * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(10, 0);
        H.set(11, 0);
        H.set(12, 0);
        H.set(13, 0);
        H.set(14, 2 * (rw * rz - rx * ry) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(15, (-Math.pow(rw, 2) + Math.pow(rx, 2) - Math.pow(ry, 2) + Math.pow(rz, 2))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(16, 2 * (-rw * rx - ry * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(17,
                (-rw * ry * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                        + rw * (rw * (rw * (-jy + y) - rx * (jz - z) + rz * (jx - x))
                                + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rw * (rw * (-rw * (jy - y) - rx * (jz - z) + rz * (jx - x))
                                - rx * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                - ry * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                                + rz * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y)))
                        + rx * (rw * (rw * (-jz + z) + rx * (jy - y) - ry * (jx - x))
                                + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        - rz * (rw * (rw * (-jx + x) + ry * (jz - z) - rz * (jy - y))
                                + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(18,
                (rw * (rx * (-rw * (jy - y) + rx * (-jz + z) + rz * (jx - x))
                        + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rx * rz * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                        - rx * (rx * (rw * (jz - z) + rx * (-jy + y) + ry * (jx - x))
                                + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rx * (-rw * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                + rx * (-rw * (jz - z) + rx * (jy - y) - ry * (jx - x))
                                - ry * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                                + rz * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y)))
                        + ry * (rx * (rx * (-jx + x) - ry * (jy - y) - rz * (jz - z))
                                + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(19,
                (-rw * ry * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                        + rx * (ry * (-rw * (jz - z) + rx * (jy - y) + ry * (-jx + x))
                                + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + ry * (ry * (-rx * (jx - x) + ry * (-jy + y) - rz * (jz - z))
                                + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + ry * (-rw * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                - rx * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                - ry * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                                + rz * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y)))
                        + rz * (ry * (rw * (jx - x) + ry * (-jz + z) + rz * (jy - y))
                                + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(20,
                (-rw * (rz * (rw * (jy - y) + rx * (jz - z) + rz * (-jx + x))
                        + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        - rx * rz * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                        + ry * (rz * (-rx * (jx - x) - ry * (jy - y) + rz * (-jz + z))
                                + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        - rz * (rz * (-rw * (jx - x) + ry * (jz - z) + rz * (-jy + y))
                                + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rz * (-rw * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                - rx * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                - ry * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                                + rz * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y)))
                        + (-rw * (jx - x) + ry * (jz - z) - rz * (jy - y))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(21, 2 * (-rw * rz + rx * ry) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(22, (Math.pow(rw, 2) - Math.pow(rx, 2) + Math.pow(ry, 2) - Math.pow(rz, 2))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(23, 2 * (rw * rx + ry * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(24, 0);
        H.set(25, 0);
        H.set(26, 0);
        H.set(27, 0);
        H.set(28, 2 * (-rw * ry - rx * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(29, 2 * (rw * rx - ry * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(30, (-Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) - Math.pow(rz, 2))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(31,
                (-rw * rz * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                        + rw * (rw * (rw * (-jz + z) + rx * (jy - y) - ry * (jx - x))
                                + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rw * (rw * (-rw * (jz - z) + rx * (jy - y) - ry * (jx - x))
                                + rx * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                - ry * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                                - rz * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z)))
                        - rx * (rw * (rw * (-jy + y) - rx * (jz - z) + rz * (jx - x))
                                + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + ry * (rw * (rw * (-jx + x) + ry * (jz - z) - rz * (jy - y))
                                + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(32,
                (-rw * (rx * (rw * (jz - z) + rx * (-jy + y) + ry * (jx - x))
                        + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        - rx * ry * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                        - rx * (rx * (-rw * (jy - y) + rx * (-jz + z) + rz * (jx - x))
                                + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rx * (-rw * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                + rx * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                - ry * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                                - rz * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z)))
                        + rz * (rx * (rx * (-jx + x) - ry * (jy - y) - rz * (jz - z))
                                + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + (-rw * (jy - y) - rx * (jz - z) + rz * (jx - x))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(33,
                (rw * (ry * (-rw * (jz - z) + rx * (jy - y) + ry * (-jx + x))
                        + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rx * ry * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                        - ry * (ry * (rw * (jx - x) + ry * (-jz + z) + rz * (jy - y))
                                + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + ry * (-rw * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                + rx * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                + ry * (-rw * (jx - x) + ry * (jz - z) - rz * (jy - y))
                                - rz * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z)))
                        + rz * (ry * (-rx * (jx - x) + ry * (-jy + y) - rz * (jz - z))
                                + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(34,
                (-rw * rz * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                        + rx * (rz * (rw * (jy - y) + rx * (jz - z) + rz * (-jx + x))
                                + (jx - x) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + ry * (rz * (-rw * (jx - x) + ry * (jz - z) + rz * (-jy + y))
                                + (jy - y) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rz * (rz * (-rx * (jx - x) - ry * (jy - y) + rz * (-jz + z))
                                + (jz - z) * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        + rz * (-rw * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                                + rx * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                                - ry * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                                - rz * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z)))
                        + (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                                * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)))
                        / Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 2));
        H.set(35, 2 * (rw * ry + rx * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(36, 2 * (-rw * rx + ry * rz) / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(37, (Math.pow(rw, 2) - Math.pow(rx, 2) - Math.pow(ry, 2) + Math.pow(rz, 2))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        H.set(38, 0);
        H.set(39, 0);
        H.set(40, 0);
        H.set(41, 0);
        H.set(42, 0);
        H.set(43, 0);
        H.set(44, 0);
        H.set(45, (jrw * Math.pow(rx, 2) + jrw * Math.pow(ry, 2) + jrw * Math.pow(rz, 2) - jrx * rw * rx - jry * rw * ry
                - jrz * rw * rz)
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(46, (-jrw * rw * rx + jrx * Math.pow(rw, 2) + jrx * Math.pow(ry, 2) + jrx * Math.pow(rz, 2)
                - jry * rx * ry - jrz * rx * rz)
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(47, (-jrw * rw * ry - jrx * rx * ry + jry * Math.pow(rw, 2) + jry * Math.pow(rx, 2)
                + jry * Math.pow(rz, 2) - jrz * ry * rz)
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(48, (-jrw * rw * rz - jrx * rx * rz - jry * ry * rz + jrz * Math.pow(rw, 2) + jrz * Math.pow(rx, 2)
                + jrz * Math.pow(ry, 2))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(49, 0);
        H.set(50, 0);
        H.set(51, 0);
        H.set(52, (-jrw * jrx * rx - jrw * jry * ry - jrw * jrz * rz + Math.pow(jrx, 2) * rw + Math.pow(jry, 2) * rw
                + Math.pow(jrz, 2) * rw)
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(53, (Math.pow(jrw, 2) * rx - jrw * jrx * rw - jrx * jry * ry - jrx * jrz * rz + Math.pow(jry, 2) * rx
                + Math.pow(jrz, 2) * rx)
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(54, (Math.pow(jrw, 2) * ry - jrw * jry * rw + Math.pow(jrx, 2) * ry - jrx * jry * rx - jry * jrz * rz
                + Math.pow(jrz, 2) * ry)
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(55, (Math.pow(jrw, 2) * rz - jrw * jrz * rw + Math.pow(jrx, 2) * rz - jrx * jrz * rx
                + Math.pow(jry, 2) * rz - jry * jrz * ry)
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(56, 0);
        H.set(57, 0);
        H.set(58, 0);
        H.set(59, (jrx * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rw * (jrw * rx - jrx * rw - jry * rz + jrz * ry))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(60, (-jrw * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rx * (jrw * rx - jrx * rw - jry * rz + jrz * ry))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(61, (-jrz * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + ry * (jrw * rx - jrx * rw - jry * rz + jrz * ry))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(62, (jry * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rz * (jrw * rx - jrx * rw - jry * rz + jrz * ry))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(63, 0);
        H.set(64, 0);
        H.set(65, 0);
        H.set(66, (jrw * (jrw * rx - jrx * rw - jry * rz + jrz * ry)
                - rx * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(67, (jrx * (jrw * rx - jrx * rw - jry * rz + jrz * ry)
                + rw * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(68, (jry * (jrw * rx - jrx * rw - jry * rz + jrz * ry)
                + rz * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(69, (jrz * (jrw * rx - jrx * rw - jry * rz + jrz * ry)
                - ry * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(70, 0);
        H.set(71, 0);
        H.set(72, 0);
        H.set(73, (jry * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rw * (jrw * ry + jrx * rz - jry * rw - jrz * rx))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(74, (jrz * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rx * (jrw * ry + jrx * rz - jry * rw - jrz * rx))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(75, (-jrw * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + ry * (jrw * ry + jrx * rz - jry * rw - jrz * rx))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(76, (-jrx * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rz * (jrw * ry + jrx * rz - jry * rw - jrz * rx))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(77, 0);
        H.set(78, 0);
        H.set(79, 0);
        H.set(80, (jrw * (jrw * ry + jrx * rz - jry * rw - jrz * rx)
                - ry * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(81, (jrx * (jrw * ry + jrx * rz - jry * rw - jrz * rx)
                - rz * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(82, (jry * (jrw * ry + jrx * rz - jry * rw - jrz * rx)
                + rw * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(83, (jrz * (jrw * ry + jrx * rz - jry * rw - jrz * rx)
                + rx * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(84, 0);
        H.set(85, 0);
        H.set(86, 0);
        H.set(87, (jrz * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rw * (jrw * rz - jrx * ry + jry * rx - jrz * rw))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(88, (-jry * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rx * (jrw * rz - jrx * ry + jry * rx - jrz * rw))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(89, (jrx * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + ry * (jrw * rz - jrx * ry + jry * rx - jrz * rw))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(90, (-jrw * (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))
                + rz * (jrw * rz - jrx * ry + jry * rx - jrz * rw))
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.pow(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2), 3.0 / 2.0)));
        H.set(91, 0);
        H.set(92, 0);
        H.set(93, 0);
        H.set(94, (jrw * (jrw * rz - jrx * ry + jry * rx - jrz * rw)
                - rz * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(95, (jrx * (jrw * rz - jrx * ry + jry * rx - jrz * rw)
                + ry * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(96, (jry * (jrw * rz - jrx * ry + jry * rx - jrz * rw)
                - rx * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        H.set(97, (jrz * (jrw * rz - jrx * ry + jry * rx - jrz * rw)
                + rw * (Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2)))
                / (Math.pow(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2), 3.0 / 2.0)
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));

        return H;
    }
}
