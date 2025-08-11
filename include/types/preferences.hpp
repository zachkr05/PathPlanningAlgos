
#pragma once
#include <string>


namespace UserPreferences {
  struct PotentialFieldParams{

			double amplitude {1.0};
			double sigma {0.5};
			std::string id;


  inline bool operator<(const PotentialFieldParams& other) const noexcept {
              // Order by id first, then amplitude, then sigma
              if (id != other.id) return id < other.id;
              if (amplitude != other.amplitude) return amplitude < other.amplitude;
              return sigma < other.sigma;
          }

	}; //struct PotentialFieldParams

} // namespace UserPreferences
