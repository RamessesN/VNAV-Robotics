#include "random_vector.h"
#include <random>
#include <numeric>
#include <iomanip>
#include <cmath>
#include <iostream>

using namespace std;

/**
 * Constructor to initialize the vector with random values
 * @param size The size of the vector
 * @param max_val The maximum value for the random numbers
 */
RandomVector::RandomVector(int size, double max_val) {
    vect.resize(size);
    for(int i = 0; i < size; i++) {
        vect[i] = (double)rand() / RAND_MAX * max_val;
    }
}

/**
 * Print the elements of the vector
 */
void RandomVector::print() {
  for(size_t i = 0; i < vect.size(); i++) {
    printf("%.6lf ", vect[i]);
    if((i + 1) % 9 == 0)
      cout << endl;
  }

  cout << endl;
}


/**
 * Calculate the mean of the vector elements
 * @return The mean value
 */
double RandomVector::mean() {
  if(vect.empty()) return 0.0;

  double sum = 0.0;
  for(size_t i = 0; i < vect.size(); i++) {
    sum += vect[i];
  }

  return sum / vect.size();
}

/**
 * Find the maximum value in the vector
 * @return The maximum value
 */
double RandomVector::max() {
  if(vect.empty()) return 0.0;

  double max_val = vect[0];
  for(size_t i = 1; i < vect.size(); i++) {
    if(vect[i] > max_val){
      max_val = vect[i];
    }
  }

  return max_val;
}

/**
 * Find the minimum value in the vector
 * @return The minimum value
 */
double RandomVector::min() {
  if(vect.empty()) return 0.0;

  double min_val = vect[0];
  for(size_t i = 1; i < vect.size(); i++) {
    if(vect[i] < min_val){
      min_val = vect[i];
    }
  }

  return min_val;
}

/**
 * Print a histogram of the vector elements
 * @param bins The number of bins in the histogram
 */
void RandomVector::printHistogram(int bins){
  if(vect.empty() || bins <= 0) return;

  double min_val = min();
  double max_val = max();
  double bin_size = (max_val - min_val) / bins;

  vector<int> counts(bins, 0); // to store counts for each bin

  for(double val : vect) {
    int index = static_cast<int>((val - min_val) / bin_size);

    if(index == bins) index--;

    counts[index]++;
  }

  int max_count = 0;
  for(int count : counts)
    if(count > max_count) max_count = count; // find the maximum count for scaling

  for(int level = max_count; level > 0; level--) {
    for(int bin = 0; bin < bins; bin++) {
      if(counts[bin] >= level)
        cout << "***";
      else
        cout << "   ";
      cout << " ";
    }
    cout << endl;
  }
}