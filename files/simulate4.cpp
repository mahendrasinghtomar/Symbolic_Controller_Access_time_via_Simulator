/* dr_api
 * simulate4.cc, stitched
 *
 */
#include "dr_api.h"
#include <assert.h>
#include <iostream>
#include <array>
#include <cmath>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"
/* time profiling */
#include "TicToc.hh"

// #define TRDIR "400"
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

#define TRDIR "GoldmontV01_"

bool
my_setenv(const char *var, const char *value)
{
#ifdef UNIX
    return setenv(var, value, 1 /*override*/) == 0;
#else
    return SetEnvironmentVariable(var, value) == TRUE;
#endif
}

int main() {

  double xshift = 9.5;
  int si = 11;  // (si+1) adjacent scenes

  std::string setenvString = "-stderr_mask 0xc -rstats_to_stderr -client_lib ';;-offline -outdir /SCOTSv0.2_Copy/examples/vehicle_mxu/build/memtrace_s3_F2_" + std::string(TRDIR) + "'";

  if (!my_setenv("DYNAMORIO_OPTIONS",  
                    setenvString.c_str()))
        std::cerr << "failed to set env var!\n";

TicToc run_time;
run_time.tic();

   /* define function to check if we are in target */
  auto target = [&si,&xshift](const state_type& x, int& i) {
    /* i updated */
    // if (9 <= x[0] && x[0] <= 9.5 && 0 <= x[1] && x[1] <= 0.5)
    if (9.5+(i*xshift) <= x[0] && x[0] <= 10+(i*xshift) && 0 <= x[1] && x[1] <= 1){
      if(i==si)
        return true;
      else
        i++;
    }
    return false;
  };

// scarab_begin(); // cycles to read static controller, together with 'if' check and 'con' creation.
  // std::vector<scots::StaticController> conVec[si+1];
  std::vector<scots::StaticController> conVec;
  for(int i=0;i<si+1;i++){
    scots::StaticController conTemp;
    /* read controller from file */
    // scots::StaticController con;
    if(!read_from_file(conTemp,EXAMPLE+"_controller"+FILENAMEMODIF+"_"+std::to_string(i))) {
      std::cout << "Could not read controller from controller.scs\n";
      return 0;
    }
    // conVec[i] = conTemp;
    conVec.push_back(conTemp);
  }
// scarab_end();
  
  std::cout << "\nSimulation:\n " << std::endl;

  // dr_app_setup();
  // assert(!dr_app_running_under_dynamorio());

  // cycles to read static controller, together with 'if' check and 'con' creation.
  // for(int ii=0;ii<5;ii++){
    int no_steps = 0;
    state_type x={{.5, 0.4, 0}};
    int i=0;
    while(1) {
      no_steps++;

      // if(no_steps==5)
      //   return 0;

      dr_app_setup();
      dr_app_start();
      // assert(dr_app_running_under_dynamorio());
      std::vector<input_type> u = conVec[i].get_control<state_type,input_type>(x);
      dr_app_stop_and_cleanup();
      // assert(!dr_app_running_under_dynamorio());

      std::cout << x[0] <<  " "  << x[1] << " " << x[2] << ", scene " << i <<  "\n";
      //std::cout << u[0][0] <<  " "  << u[0][1] << "\n";
      vehicle_post(x,u[0]);
      if(target(x,i)) {
        std::cout << "Arrived: " << x[0] <<  " "  << x[1] << " " << x[2] << std::endl;
        break;
      }
    }
    
    std::cout << "Vehicle Run time = ";
    run_time.toc();
    std::cout << " Number of steps = " << no_steps << "\n" ;

    // dr_app_stop_and_cleanup();
    // assert(!dr_app_running_under_dynamorio());

  // }

  return 0;
}
