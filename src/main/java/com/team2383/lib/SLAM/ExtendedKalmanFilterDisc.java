package com.team2383.lib.SLAM;

import org.ejml.simple.SimpleMatrix;

import java.util.function.BiFunction;

public class ExtendedKalmanFilterDisc {
    private SimpleMatrix mu;
    private SimpleMatrix sigma;
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> g_t;
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> G_t;

    public ExtendedKalmanFilterDisc(
            SimpleMatrix init_mu, SimpleMatrix init_sigma,
            BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> g_t,
            BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> G_t,
            BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> h_t,
            BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> H_t) {
        this.mu = init_mu;
        this.sigma = init_sigma;
        this.g_t = g_t;
        this.G_t = G_t;
    }

    public void predict(SimpleMatrix u_t, SimpleMatrix R) {
        mu = g_t.apply(u_t, mu);

        SimpleMatrix G = G_t.apply(u_t, mu);
        sigma = G.mult(sigma).mult(G.transpose()).plus(R);
    }

    public SimpleMatrix mu() {
        return mu;
    }
}
