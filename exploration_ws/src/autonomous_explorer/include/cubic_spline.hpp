#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cubic_spline {

class CubicSpline {
public:
    CubicSpline() {}
    
    void initialize(const std::vector<double>& x, const std::vector<double>& y) {
        if (x.size() != y.size()) {
            throw std::invalid_argument("x and y vec nopt same size");
        }
        
        if (x.size() < 2) {
            throw std::invalid_argument("need 2 points");
        }
        
        for (size_t i = 1; i < x.size(); ++i) {
            if (x[i] <= x[i-1]) {
                throw std::invalid_argument("x need to be inc");
            }
        }
        
        x_ = x;
        y_ = y;
        
        size_t n = x_.size() - 1;
        
        std::vector<double> h(n);
        for (size_t i = 0; i < n; ++i) {
            h[i] = x_[i+1] - x_[i];
        }
        
        std::vector<double> alpha(n);
        for (size_t i = 1; i < n; ++i) {
            alpha[i] = 3.0 / h[i] * (y_[i+1] - y_[i]) - 3.0 / h[i-1] * (y_[i] - y_[i-1]);
        }
        
        std::vector<double> l(n+1, 0.0);
        std::vector<double> mu(n+1, 0.0);
        std::vector<double> z(n+1, 0.0);
        
        for (size_t i = 1; i < n; ++i) {
            l[i] = 2.0 * (x_[i+1] - x_[i-1]) - h[i-1] * mu[i-1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
        }
        
        a_ = y_;
        b_.resize(n+1);
        c_.resize(n+1);
        d_.resize(n+1);
        
        c_[n] = 0.0;
        for (int i = n-1; i >= 0; --i) {
            c_[i] = z[i] - mu[i] * c_[i+1];
            b_[i] = (a_[i+1] - a_[i]) / h[i] - h[i] * (c_[i+1] + 2.0 * c_[i]) / 3.0;
            d_[i] = (c_[i+1] - c_[i]) / (3.0 * h[i]);
        }
    }
    
    double evaluate(double x) const {
        if (x_.empty()) {
            throw std::runtime_error("Splinebasd");
        }
        
        size_t i = 0;
        if (x < x_[0]) {
            i = 0;
        } else if (x >= x_[x_.size() - 1]) {
            i = x_.size() - 2;
        } else {
            auto it = std::upper_bound(x_.begin(), x_.end(), x);
            i = std::distance(x_.begin(), it) - 1;
        }
        
        double dx = x - x_[i];
        return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }
    
    double evaluate_derivative(double x) const {
        if (x_.empty()) {
            throw std::runtime_error("empty");
        }
        
        size_t i = 0;
        if (x < x_[0]) {
            i = 0;
        } else if (x >= x_[x_.size() - 1]) {
            i = x_.size() - 2;
        } else {
            auto it = std::upper_bound(x_.begin(), x_.end(), x);
            i = std::distance(x_.begin(), it) - 1;
        }
        
        double dx = x - x_[i];
        return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
    }
    
    double evaluate_second_derivative(double x) const {
        if (x_.empty()) {
            throw std::runtime_error("Spline not initialized");
        }
        
        size_t i = 0;
        if (x < x_[0]) {
            i = 0;
        } else if (x >= x_[x_.size() - 1]) {
            i = x_.size() - 2;
        } else {
            auto it = std::upper_bound(x_.begin(), x_.end(), x);
            i = std::distance(x_.begin(), it) - 1;
        }
        
        double dx = x - x_[i];
        return 2.0 * c_[i] + 6.0 * d_[i] * dx;
    }
    
    const std::vector<double>& get_x() const {
        return x_;
    }
    
    const std::vector<double>& get_y() const {
        return y_;
    }
    
private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
};

class Spline2D {
public:
    Spline2D() {}
    
    void initialize(const std::vector<double>& x, const std::vector<double>& y) {
        std::vector<double> s(x.size(), 0.0);
        for (size_t i = 1; i < x.size(); ++i) {
            double dx = x[i] - x[i-1];
            double dy = y[i] - y[i-1];
            s[i] = s[i-1] + std::sqrt(dx*dx + dy*dy);
        }
        
        sx_.initialize(s, x);
        sy_.initialize(s, y);
        
        s_ = s;
    }
    
    std::pair<double, double> calculate_position(double s) const {
        double x = sx_.evaluate(s);
        double y = sy_.evaluate(s);
        return {x, y};
    }
    
    double calculate_curvature(double s) const {
        double dx = sx_.evaluate_derivative(s);
        double dy = sy_.evaluate_derivative(s);
        double ddx = sx_.evaluate_second_derivative(s);
        double ddy = sy_.evaluate_second_derivative(s);
        
        double norm = std::pow(dx*dx + dy*dy, 1.5);
        if (norm < 1e-10) {
            return 0.0;
        }
        
        return (dx*ddy - dy*ddx) / norm;
    }
    
    double calculate_yaw(double s) const {
        double dx = sx_.evaluate_derivative(s);
        double dy = sy_.evaluate_derivative(s);
        return std::atan2(dy, dx);
    }
    
    double get_total_length() const {
        return s_.back();
    }
    
    const std::vector<double>& get_s() const {
        return s_;
    }
    
private:
    CubicSpline sx_;
    CubicSpline sy_;
    std::vector<double> s_;
};

std::vector<std::pair<double, double>> smooth_path(
    const std::vector<std::pair<double, double>>& path, 
    double step_size = 0.1) {
    
    if (path.size() < 2) {
        return path;
    }
    
    std::vector<double> x, y;
    for (const auto& point : path) {
        x.push_back(point.first);
        y.push_back(point.second);
    }
    
    Spline2D spline;
    spline.initialize(x, y);
    
    std::vector<std::pair<double, double>> smoothed_path;
    double total_length = spline.get_total_length();
    
    for (double s = 0.0; s <= total_length; s += step_size) {
        auto point = spline.calculate_position(s);
        smoothed_path.push_back(point);
    }
    
    if (smoothed_path.empty() || 
        (smoothed_path.back().first != path.back().first || 
         smoothed_path.back().second != path.back().second)) {
        smoothed_path.push_back(path.back());
    }
    
    return smoothed_path;
}

} // namespace cubic_spline

#endif // CUBIC_SPLINE_HPP
