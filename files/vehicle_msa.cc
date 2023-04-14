/* msa: multiple scenarios adjacent
 * vehicle.cc
 *
 *  created: Oct 2016
 *   author: Matthias Rungger
 */

#include <iostream>
#include <array>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"

/* time profiling */
#include "TicToc.hh"
/* memory profiling */
#include <sys/time.h>
#include <sys/resource.h>
struct rusage usage;

/* state space dim */
const int state_dim=3;
/* input space dim */
const int input_dim=2;

/* sampling time */
const double tau = 0.3;

std::string FILENAMEMODIF = "_msa";
const std::string EXAMPLE = "vehicle";

/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

/* abbrev of the type for abstract states and inputs */
using abs_type = scots::abs_type;

/* we integrate the vehicle ode by tau sec (the result is stored in x)  */
auto  vehicle_post = [](state_type &x, const input_type &u) {
  /* the ode describing the vehicle */
  auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) {
    double alpha=std::atan(std::tan(u[1])/2.0);
    xx[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
    xx[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
    xx[2] = u[0]*std::tan(u[1]);
  };
  /* simulate (use 10 intermediate steps in the ode solver) */
  scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);
};

/* we integrate the growth bound by 0.3 sec (the result is stored in r)  */
auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
  double c = std::abs(u[0])*std::sqrt(std::tan(u[1])*std::tan(u[1])/4.0+1);
  r[0] = r[0]+c*r[2]*tau;
  r[1] = r[1]+c*r[2]*tau;
};

int main() {

  double xshift = 9.5;
  double obs_width = 0.26;
  for(int si=0;si<12;si++){
  /* to measure time */
  TicToc tt;
  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  state_type s_lb={{0+(si*xshift),0,-3.5}};
  /* upper bounds of the hyper rectangle */
  state_type s_ub={{10+(si*xshift),10,3.5}};
  /* grid node distance diameter */
  state_type s_eta={{.02,.02,0.18}};
  // state_type s_eta={{.1,.1,0.18}};
  scots::UniformGrid ss(state_dim,s_lb,s_ub,s_eta);
  std::cout << "Uniform grid details:" << std::endl;
  ss.print_info();

  scots::write_to_file(ss,EXAMPLE+"_state_grid"+FILENAMEMODIF);
  
  /* construct grid for the input space */
  /* lower bounds of the hyper rectangle */
  input_type i_lb={{-1,-1}};
  /* upper bounds of the hyper rectangle */
  input_type i_ub={{ 1, 1}};
  /* grid node distance diameter */
  input_type i_eta={{.3,.3}};
  scots::UniformGrid is(input_dim,i_lb,i_ub,i_eta);
  is.print_info();

  std::srand(std::time(nullptr)); // random number seed
  double o1l = 1.0 + (si*xshift) + rand() % 6;
  int for_o2l = std::floor(9.0+(si*xshift) - (o1l+obs_width+1.5));
  double o2l = o1l+obs_width + 1.5 + rand() % for_o2l;
  /* set up constraint functions with obtacles */
  double H[2][4] = {
    // { 1  , 1.2, 0  ,   9 },
    // { 2.2, 2.4, 0  ,   5 },
    // { 2.2, 2.4, 6  ,  10 },
    { o1l, o1l+obs_width, 0  ,   9 },
    { o2l, o2l+obs_width, 1  ,  10 }
    // { 5.8, 6  , 0  ,   6 },
    // { 5.8, 6  , 7  ,  10 },
    // { 7  , 7.2, 1  ,  10 },
    // { 8.2, 8.4, 0  ,  8.5},
    // { 8.4, 9.3, 8.3,  8.5},
    // { 9.3, 10 , 7.1,  7.3},
    // { 8.4, 9.3, 5.9,  6.1},
    // { 9.3, 10 , 4.7,  4.9},
    // { 8.4, 9.3, 3.5,  3.7},
    // { 9.3, 10 , 2.3,  2.5}
  };

  /* avoid function returns 1 if x is in avoid set  */
  auto avoid = [&H,ss,s_eta](const abs_type& idx) {
    state_type x;
    ss.itox(idx,x);
    double c1= s_eta[0]/2.0+1e-10;
    double c2= s_eta[1]/2.0+1e-10;
    for(size_t i=0; i<2; i++) {
      if ((H[i][0]-c1) <= x[0] && x[0] <= (H[i][1]+c1) && 
          (H[i][2]-c2) <= x[1] && x[1] <= (H[i][3]+c2))
        return true;
    }
    return false;
  };
  // auto avoid = [ss,s_eta](const abs_type& idx) {
  //   state_type x;
  //   ss.itox(idx,x);
  //   // double c1= s_eta[0]/2.0+1e-10;
  //   // double c2= s_eta[1]/2.0+1e-10;
  //   if (((x[0]-5)*(x[0]-5)+(x[1]-5)*(x[1]-5) < 9) || ((x[0]-5)*(x[0]-5)+(x[1]-5)*(x[1]-5) > 25)) 
  //     return true;
  //   if (x[0] < 2.2 && 4.8 < x[1] && x[1] < 5.2)
  //     return true;
  //   return false;
  // };
  /* write obstacles to file */
  write_to_file(ss,avoid,EXAMPLE+"_obstacles"+FILENAMEMODIF+"_"+std::to_string(si));

  std::cout << "Computing the transition function: " << std::endl;
  /* transition function of symbolic model */
  scots::TransitionFunction tf;
  scots::Abstraction<state_type,input_type> abs(ss,is);

  tt.tic();
  abs.compute_gb(tf,vehicle_post, radius_post, avoid);
  //abs.compute_gb(tf,vehicle_post, radius_post);
  tt.toc();

  if(!getrusage(RUSAGE_SELF, &usage))
    std::cout << "Memory per transition: " << usage.ru_maxrss/(double)tf.get_no_transitions() << std::endl;
  std::cout << "Number of transitions: " << tf.get_no_transitions() << std::endl;

  /* define target set */
  auto target = [&ss,&s_eta,&si,&xshift](const abs_type& idx) {
    state_type x;
    ss.itox(idx,x);
    /* function returns 1 if cell associated with x is in target set  */
    if (9.5+(si*xshift) <= (x[0]-s_eta[0]/2.0) && (x[0]+s_eta[0]/2.0) <= 10+(si*xshift) && 
        0 <= (x[1]-s_eta[1]/2.0) && (x[1]+s_eta[1]/2.0) <= 1)
      return true;
    return false;
  };
   /* write target to file */
  write_to_file(ss,target,EXAMPLE+"_target"+FILENAMEMODIF+"_"+std::to_string(si));

 
  std::cout << "\nSynthesis: " << std::endl;
  tt.tic();
  scots::WinningDomain win=scots::solve_reachability_game(tf,target);
  tt.toc();
  std::cout << "Winning domain size: " << win.get_size() << std::endl;

  std::cout << "\nWrite controller to controller.scs \n";
  if(write_to_file(scots::StaticController(ss,is,std::move(win)),EXAMPLE+"_controller"+FILENAMEMODIF+"_"+std::to_string(si)))
    std::cout << "Done. \n";
  }
  return 0;
}
