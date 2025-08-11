

#include "types/preferences.hpp"
#include "types/obstacle.hpp"
#include <iostream>


int main(){


  Environment::Obstacle obstacle = {"Chair 1", Eigen::Vector2d(2,2), 5};
  
  UserPreferences::PotentialFieldParams preference = {4,5,"Chairs"};

  std::cout << preference.sigma << std::endl;
  std::cout << preference.id << std::endl;
  std::cout << preference.amplitude << std::endl;
  



  std::cout << obstacle.id << std::endl;
  std::cout << obstacle.center << std::endl;
  std::cout << obstacle.radius << std::endl;


}
