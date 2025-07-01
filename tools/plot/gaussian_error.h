#pragma once
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
template <typename T>
class GaussianError {
 public:
  GaussianError(T value = 0, T uncertainty = 0) : value_(value), uncertainty_(uncertainty) {}

  GaussianError<T> operator+(const GaussianError<T>& other) const {
    return GaussianError<T>(value_ + other.value_, sqrt(uncertainty_ * uncertainty_ +
                                                        other.uncertainty_ * other.uncertainty_));
  }

  GaussianError<T> operator-(const GaussianError<T>& other) const {
    return GaussianError<T>(value_ - other.value_, sqrt(uncertainty_ * uncertainty_ +
                                                        other.uncertainty_ * other.uncertainty_));
  }

  GaussianError<T> operator*(const GaussianError<T>& other) const {
    T resultValue = value_ * other.value_;
    T resultUncertainty = resultValue * sqrt(pow(uncertainty_ / value_, 2) +
                                             pow(other.uncertainty_ / other.value_, 2));
    return GaussianError<T>(resultValue, resultUncertainty);
  }

  GaussianError<T> operator/(const GaussianError<T>& other) const {
    if (other.value_ == 0) {
      throw std::exception();
    }
    T resultValue = value_ / other.value_;
    T resultUncertainty = resultValue * sqrt(pow(uncertainty_ / value_, 2) +
                                             pow(other.uncertainty_ / other.value_, 2));
    return GaussianError<T>(resultValue, resultUncertainty);
  }
  auto operator<=>(const GaussianError<T>& other) const { return value_ <=> other.value_; }
  bool operator==(const double val) const { return value_ == val; }
  bool operator==(const GaussianError<T>& val) const { return value_ == val; }

  // Getter methods
  T getValue() const { return value_; }

  T stdev() const { return uncertainty_; }

  // Print method
  void print() const {
    std::cout << "Value: " << value_ << ", Uncertainty: " << uncertainty_ << std::endl;
  }
  std::string to_string(bool stdev = false) const {
    std::stringstream res;

    if (!stdev) {
      res << std::fixed << std::setprecision(2) << value_;
    } else {
      res << (*this);
    }
    return res.str();
  }

 private:
  T value_;
  T uncertainty_;
};
template <typename T>
std::ostream& operator<<(std::ostream& os, const GaussianError<T>& obj) {
  // write obj to stream
  os << obj.getValue() << " (stdev=" << obj.stdev() << " )";
  return os;
}