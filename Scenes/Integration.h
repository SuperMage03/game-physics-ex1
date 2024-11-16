#pragma once
namespace Integration {
    template<typename Integrand, typename Lambda>
    static Integrand eulerIntegrate(const Integrand& y, float x, const Lambda&& f, float delta) {
        return y + delta * f(x, y);
    }

    template<typename Integrand, typename Lambda>
    static Integrand midpointIntegrate(const Integrand& y, float x, const Lambda&& f, float delta) {
        Integrand midpoint = y + (delta / 2.f) * f(x, y);
        return y + delta * f(x + delta / 2.f, midpoint);
    }

    template<typename Integrand, typename Lambda>
    static Integrand RK4Integrate(const Integrand& y, float x, const Lambda&& f, float delta) {
        Integrand k1 = f(x, y);
        Integrand k2 = f(x + delta / 2.f, y + (delta / 2.) * k1);
        Integrand k3 = f(x + delta / 2.f, y + (delta / 2.) * k2);
        Integrand k4 = f(x + delta, y + delta * k3);
        return y + (delta / 6.f) * (k1 + 2.f * k2 + 2.f * k3 + k4);
    }

    template<typename Integrand, typename Lambda>
    static Integrand eulerIntegrate(const Integrand& y, const Lambda&& f, float delta) {
        return y + delta * f(y);
    }

    template<typename Integrand, typename Lambda>
    static Integrand midpointIntegrate(const Integrand& y, const Lambda&& f, float delta) {
        Integrand midpoint = y + (delta / 2.f) * f(y);
        return y + delta * f(midpoint);
    }

    template<typename Integrand, typename Lambda>
    static Integrand RK4Integrate(const Integrand& y, const Lambda&& f, float delta) {
        Integrand k1 = f(y);
        Integrand k2 = f(y + (delta / 2.f) * k1);
        Integrand k3 = f(y + (delta / 2.f) * k2);
        Integrand k4 = f(y + delta * k3);
        return y + (delta / 6.f) * (k1 + 2.f * k2 + 2.f * k3 + k4);
    }
}