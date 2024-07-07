# Manifold-RTS-Smoother
A Manifold version of the Popular Rauch Tung Striebel Smoother. Uses the  EKF Style Update formulas of the RTS-Smoother. 
It is based on the boxplus ([+]) method of "Integrating generic sensor fusion algorithms with sound state representations through encapsulation of manifolds" Hertzberg et. al.
Thus, we call it the [+]-EKS. 



## Requirements
The [+]-EKS is based on the ADEKF: https://github.com/TomLKoller/ADEKF
It is simply a Header that does not need to be compiled, but it requires the Headers of the ADEKF repository to function.
Follow the installation instructions of the ADEKF to setup all required libraries. 

## Usage
The [+]-EKS uses a forward filtering of the state followed by a backwards smoother. It makes use of the C++ lambda syntax to define models easily.

You can find a working application of the [+]-EKS at: https://github.com/TomLKoller/Boxplus-IMM/blob/master/immBodyDistanceSmooth.cpp
Feel free to contact me if you need a minimal example. 

### Initialization/Setup
The [+]-EKS is a child class of the ADEKF. You can either pass an ADEKF object to the constructor or use the constructor of the ADEKF (state and covariance) for inizialization. See https://github.com/TomLKoller/ADEKF#usage for the general usage of the ADEKF and the definition of manifold states. Browse the ADEKF repository for some examples.
```c++
//Initialization and Model Setup
adekf::RTS_Smoother smoother{start_state,start_covariance};
auto dynamicModel=[](auto & state, auto noise, ControlTypeA a, ControlTypeB b){state=...;};
adekf::SquareMatrixType<double,NOISE_DOF> dynamic_noise{...};
auto measurementModel=[](auto state) {return ...;};
adekf::SquareMatrixType<double,MEAS_DOF> measurement_noise{...};
```
### Forward Filtering
The forward filtering can be implemented almost as in the ADEKF.
It is required to store the predicted and updated states. The [+]-EKS can handle this by calling storePredictedEstimation() and storeEstimation().
The controls of the dynamic model are required to smooth the estimate later. You can have an arbitrary number of control inputs. Store them during the forward pass in a vector of tuples:
```c++
//Before loop: Setup of Models 
std::vector<std::tuple<ControlTypeA,ControlTypeB>> all_controls;

//One iteration of the loop
ControlTypeA a=...;
ControlTypeB b=...;
smoother.predictWithNonAdditiveNoise(dynamicModel, dynamic_noise, a,b);
smoother.storePredictedEstimation();
all_controls.emplace_back(a,b); //Automatically constructs the tuple of the controls
//update
Measurement measurement=...;
smoother.update(measurementModel,measurement_noise,measurement);
smoother.storeEstimation();
```
The stored estimates are available through std::vectors named:
1.old_mus (state after update, old_mus[0] is the start state)
2.old_sigmas (covariance after update)
3.predicted_mus (state after predict)
4.predicted_sigmas (covariance after predict)

### Backwards Smoothing
Two options are available for smoothing:
1. smoothIntervalWithNonAdditiveNoise smoothes a given intervall 
2. smoothAllWithNonAdditiveNoise smoothes all filtered states

Both require the dynamicModel function, the dynamic Noise and the stored controls values:
```c++
smoother.smoothIntervalWithNonAdditiveNoise(steps, start, dynamicModel,dynamic_noise,all_controls);
// or
smoother.smoothAllWithNonAdditiveNoise(dynamicModel, dynamic_noise, all_controls);
```
The smoothed values are available through std::vectors named:
1. smoothed_mus (the smoothed states)
2. smoothed_sigmas (the smoothed covariances)

Before smoothing, the vector elements are the same as old_mus.

