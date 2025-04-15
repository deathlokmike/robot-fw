class KalmanFilter {
   private:
    double R;
    double Q;
    double P;
    double U_hat;
    double H;

   public:
    KalmanFilter(double process_noise = 10, double measurement_noise = 40,
                 double initial_estimate = 0)
        : Q(process_noise),
          R(measurement_noise),
          P(0),
          U_hat(initial_estimate),
          H(1.0) {}

    double update(double U);
};