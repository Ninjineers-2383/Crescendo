package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

public class SensorModelTag implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
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

        SimpleMatrix h = new SimpleMatrix(7, 1);

        h.set(0, (rw * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                + rx * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                - ry * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                + rz * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x)))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        h.set(1, (rw * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                + rx * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                + ry * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z))
                - rz * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y)))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        h.set(2, (rw * (rw * (jz - z) - rx * (jy - y) + ry * (jx - x))
                - rx * (rw * (jy - y) + rx * (jz - z) - rz * (jx - x))
                + ry * (rw * (jx - x) - ry * (jz - z) + rz * (jy - y))
                + rz * (rx * (jx - x) + ry * (jy - y) + rz * (jz - z)))
                / (Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2)));
        h.set(3, (jrw * rw + jrx * rx + jry * ry + jrz * rz)
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        h.set(4, (-jrw * rx + jrx * rw + jry * rz - jrz * ry)
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        h.set(5, (-jrw * ry - jrx * rz + jry * rw + jrz * rx)
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));
        h.set(6, (-jrw * rz + jrx * ry - jry * rx + jrz * rw)
                / (Math.sqrt(Math.pow(jrw, 2) + Math.pow(jrx, 2) + Math.pow(jry, 2) + Math.pow(jrz, 2))
                        * Math.sqrt(Math.pow(rw, 2) + Math.pow(rx, 2) + Math.pow(ry, 2) + Math.pow(rz, 2))));

        return h;
    }
}
