// clang: MatousFormat
/**  \file
     \brief Example file for the Repredictor implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     See \ref example.cpp.
 */

/**  \example "example.cpp"

 */

// Include the Repredictor header
#include "repredictor/repredictor.h"
// As a model, we'll use a LKF variant
#include "repredictor/lkf.h"
#include "repredictor/utils.h"

#include <chrono>
#include <random>
#include <fstream>
#include <mutex>
#include <atomic>
#include <thread>

// Define the LKF we will be using
namespace repredictor
{
  const int n_states = 2;
  const int n_inputs = 1;
  const int n_measurements = 1;

  using lkf_t = varstepLKF<n_states, n_inputs, n_measurements>;
  using rep_t = Repredictor<lkf_t>;
}

// | ------------------- HELPER DEFINITIONS ------------------- |
/* helper aliases and definitions //{ */

// Some helpful aliases to make writing of types shorter
using namespace std::chrono_literals;
using time_point = repredictor::time_point;
using seconds = repredictor::seconds;

using namespace repredictor;
using A_t = lkf_t::A_t;
using B_t = lkf_t::B_t;
using H_t = lkf_t::H_t;
using Q_t = lkf_t::Q_t;
using x_t = lkf_t::x_t;
using P_t = lkf_t::P_t;
using u_t = lkf_t::u_t;
using z_t = lkf_t::z_t;
using R_t = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

static std::random_device rd{};
static std::mt19937 gen{rd()};
static std::normal_distribution<> d{0,1};

// Helper function to generate a random Eigen matrix with normal distribution
template <int rows>
Eigen::Matrix<double, rows, 1> multivariate_gaussian(const Eigen::Matrix<double, rows, rows>& cov)
{
    Eigen::Matrix<double, rows, 1> ret;
    for (int row = 0; row < rows; row++)
      ret(row, 0) = d(gen);
    return cov*ret;
}

enum type_e
{
  input,
  proc_noise,
  meas_fast,
  meas_slow
};

std::string type_as_string(const type_e type)
{
  switch (type)
  {
    case input: return "\033[1;34minput change\033[0m";
    case proc_noise: return "\033[1;36mprocess noise change\033[0m";
    case meas_fast: return "\033[1;32mfast, imprecise measurement\033[0m";
    case meas_slow: return "\033[1;31mslow, precise measurement\033[0m";
    default: return "??wtf??";
  }
}

struct stamped_value_t
{
  time_point stamp;
  double value;
};

//}

// | ------------------- MODEL OF THE SYSTEM ------------------ |
/* model of the system //{ */

A_t generateA(const double dt)
{
  A_t A;
  A << 1, dt,
       0, 1;
  return A;
}

B_t generateB([[maybe_unused]] const double dt)
{
  B_t B;
  B << dt*dt/2.0,
       dt;
  return B;
}

const Q_t Q = 2.5*Q_t::Identity();
const H_t H( (H_t() << 1, 0).finished() );
const R_t R_fast = 5.5*R_t::Identity();
const R_t R_slow = 0.01*R_t::Identity();

const x_t x0 = x_t::Zero();
const P_t P0 = 5.0*P_t::Identity();
const u_t u0 = u_t::Zero();
const time_point t0 = std::chrono::system_clock::now();
const std::shared_ptr<lkf_t> lkf_ptr = std::make_shared<lkf_t>(generateA, generateB, H);
const unsigned buf_sz = 100;

//}

// | -------------------- GLOBAL VARIABLES -------------------- |
// the repredictor object itself
std::mutex rep_mtx;
rep_t rep(x0, P0, u0, Q, t0, lkf_ptr, buf_sz);

// current state (ground-truth), system input and their mutex
std::mutex xu_mtx;
x_t x = x_t::Random();
u_t u = u_t::Random();

// a stopping flag
std::atomic_bool terminate;

// --------------------------------------------------------------
// |                     THE IMPORTANT PART                     |
// --------------------------------------------------------------
void process_msg(const stamped_value_t msg, type_e type)
{
  // multithreading synchronization
  std::scoped_lock lck(rep_mtx);

  // update the Repredictor with the latest message based on its type
  switch (type)
  {
    case input: // interpret msg as system input
      rep.addInputChange(u_t(msg.value), msg.stamp);
      break;
    case proc_noise: // interpret msg as system input
      rep.addProcessNoiseChange(x_t(msg.value, msg.value).asDiagonal(), msg.stamp);
      break;
    case meas_fast: // interpret msg as fast measurement (use the corresponding R)
      rep.addMeasurement(z_t(msg.value), R_fast, msg.stamp);
      break;
    case meas_slow: // interpret msg as slow measurement (use the corresponding R)
      rep.addMeasurement(z_t(msg.value), R_slow, msg.stamp);
      break;
  }

  // estimate the current state using Repredictor
  const auto stamp = std::chrono::system_clock::now();
  const x_t x_pred = rep.predictTo(stamp).x;

  const double msg_delay = std::chrono::duration_cast<seconds>(stamp - msg.stamp).count();
  const auto [x_gt, u_gt] = utils::get_mutexed(xu_mtx, x, u);
  const double t = std::chrono::duration_cast<seconds>(stamp - t0).count();
  std::cout << "update at t = " << t << "s:  (" << type_as_string(type) << ": " << msg.value << ", delay: " << msg_delay << "s)\n";
  std::cout << "\tu:      " << u_gt.transpose() << "\n";
  std::cout << "\tx_gt:   " << x_gt.transpose() << "\n";
  std::cout << "\tx_pred: " << x_pred.transpose() << "\n";
  std::cout << "\tx_diff: " << (x-x_pred).cwiseAbs().transpose() << "\n";
}

// | -------------------- FAST MEASUREMENTS ------------------- |
/* generator thread of fast, imprecise measurements //{ */

void meas_generator_fast()
{
  const R_t R = R_fast;
  const double delay_std = 0.1;

  while (true)
  {
    const x_t x_gt = utils::get_mutexed(xu_mtx, x);
    const z_t z = H*x_gt + multivariate_gaussian(R);
    const time_point stamp = std::chrono::system_clock::now();
    const double delay = delay_std*std::abs(d(gen));
    std::this_thread::sleep_for(seconds(delay));
    if (terminate)
      break;

    const stamped_value_t msg {stamp, z.x()};
    process_msg(msg, type_e::meas_fast);
  }
}

//}

// | -------------------- SLOW MEASUREMENTS ------------------- |
/* generator thread of slow, precise measurements //{ */

void meas_generator_slow()
{
  const R_t R = R_slow;
  const double delay_std = 1.0;

  while (true)
  {
    const x_t x_gt = utils::get_mutexed(xu_mtx, x);
    const z_t z = H*x_gt + multivariate_gaussian(R);
    const time_point stamp = std::chrono::system_clock::now();
    const double delay = delay_std*std::abs(d(gen));
    std::this_thread::sleep_for(seconds(delay));
    if (terminate)
      break;

    const stamped_value_t msg {stamp, z.x()};
    process_msg(msg, type_e::meas_slow);
  }
}

//}

// | -------------------- THE MAIN FUNCTION ------------------- |
int main(const int argc, const char* argv[])
{
  // the user may specify the number of iterations as the program's argument
  int n_its = 100;
  try
  {
    if (argc > 1)
      n_its = std::stoi(argv[1]);
  }
  catch (...) {}

  // measurement generators (simulating sensors)
  std::thread th_fast(meas_generator_fast);
  std::thread th_slow(meas_generator_slow);

  // sleep duration for the system model time step
  const auto dur = 100ms;
  time_point prev_t = t0;

  // | ---------------- SYSTEM STATE PROPAGATION ---------------- |
  // our system model loop, generating the current state x
  for (int it = 0; it < n_its; it++)
  {
    // generate a random new system input
    const u_t u_new = 3.0*u_t::Random();
    // update the repredictor
    const stamped_value_t msg {std::chrono::system_clock::now(), u.x()};
    process_msg(msg, type_e::input);

    std::this_thread::sleep_for(dur);

    // get the duration in seconds as a double
    const time_point cur_t = std::chrono::system_clock::now();
    const auto dt = std::chrono::duration_cast<seconds>(cur_t - prev_t).count();
    prev_t = cur_t;
    // scale the process noise matrix by the duration
    const Q_t dtQ = dt*Q;
    // generate a new state
    const x_t x_old = utils::get_mutexed(xu_mtx, x);
    const x_t x_new = generateA(dt)*x_old + generateB(dt)*u_new + multivariate_gaussian(dtQ);
    utils::set_mutexed(xu_mtx, std::make_tuple(x_new, u_new), std::forward_as_tuple(x, u));
  }

  terminate = true;

  th_fast.join();
  th_slow.join();
}


