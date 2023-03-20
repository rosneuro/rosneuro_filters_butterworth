# ROSNeuro Butterworth filter

This ROSNeuro filter plugin implements a IIR Butterworth filter for real-time operations. It directly exploits the [rtfilter library](https://packages.debian.org/search?keywords=librtfilter-dev).

## Requirements:
- [librtfilter](https://packages.debian.org/search?keywords=librtfilter-dev)

## Algorithm:
The filter allow to implement a low-pass or high-pass Butterworth filter with custom order and cutoff frequency. It exploits the real-time implementation of [library rtfilter](https://packages.debian.org/search?keywords=librtfilter-dev). The filter has internal memory (provided by rtfilter) and it allows to apply the filter sample by sample.

## Usage:
The filter can be configured by means of the class constructor (*Butterworth\<T\>::Butterworth(ButterType type, int order, double cutoff, double samplerate)*) or by yaml configuration and parameter server. In the case the filter is used without being configured, it will throws a *std::runtime* exception. 

For usability, the internal resources of the filter are set during the first call to the function *Eigen::Matrix\<T\, Eigen::Dynamic, Eigen::Dynamic\> apply(const Eigen::Matrix\<T\, Eigen::Dynamic, Eigen::Dynamic\>& in)*. This effectively reduces the performances during the first call, but it allows to deduce the number of channels at runtime.


## Example of Butterworth filters in a simulated online loop
```cpp
#include <ros/ros.h>
#include "rosneuro_filters_butterworth/Butterworth.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "butterworth_simloop_config");
  const int framesize = 32;

  // YAML configuration for the filters
  rosneuro::Filter<double>* butter_lp = new rosneuro::Butterworth<double>();
  if(butter_lp->configure("ButterworthLowPass") == false) {
    ROS_ERROR("[%s] Lowpass filter configuration failed", butter_lp->name().c_str());
    return false;
  }
  ROS_INFO("[%s] Lowpass filter configuration succeeded", butter_lp->name().c_str());
	
  rosneuro::Filter<double>* butter_hp = new rosneuro::Butterworth<double>();
  if(butter_hp->configure("ButterworthHighPass") == false) {
    ROS_ERROR("[%s] Highpass filter configuration failed", butter_hp->name().c_str());
    return false;
  }
  ROS_INFO("[%s] Highpass filter configuration succeeded", butter_hp->name().c_str());
 
  // Load input data (we assume to have the function readCVS to load the data from CSV file)
  rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);
  
  // Get number of samples and channels
  int nsamples  = input.rows();
  int nchannels = input.cols();

  // Allocate matrix for lowpass and highpass
  rosneuro::DynamicMatrix<double> outlp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
  rosneuro::DynamicMatrix<double> outhp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	
  // Allocate frame data (for simulating online loop) 	
  rosneuro::DynamicMatrix<double> framedata = rosneuro::DynamicMatrix<double>::Zero(framesize, nchannels);
  
  // Start iteration for simulating online loop
  ROS_INFO("Start simulated loop");
  for(auto i = 0; i<nsamples; i = i+framesize) {
    framedata = input.middleRows(i, framesize);
    outlp.middleRows(i, framesize) = butter_lp->apply(framedata);
    outhp.middleRows(i, framesize) = butter_hp->apply(framedata);
  }

  ros::shutdown();
  
  return 0;
}

```
The corresponding YAML configuration file is:
```
ButterworthLowPass:
  name: butterworth
  type: ButterworthFilterDouble
  params: 
    samplerate: 512
    type: lowpass
    order: 4
    cutoff: 10

ButterworthHighPass:
  name: butterworth
  type: ButterworthFilterDouble
  params: 
    samplerate: 512
    type: highpass
    order: 4
    cutoff: 1
```
## Example of bandpass Butterworth filters (snippet)
In order to implement a bandpass butterworth filter, it is possible to exploit the *FilterChain\<T\>* class as illustrated below:
```cpp
#include <ros/ros.h>
#include <rosneuro_filters/FilterChain.hpp>
#include "rosneuro_filters_butterworth/Butterworth.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "butterworth_simloop_chain");

  // YAML configuration for the filter chain
  rosneuro::FilterChain<double> butter_bp;
  if(butter_bp.configure("ButterworthBandPass") == false) {
    ROS_ERROR("[FilterChain] Bandpass filter configuration failed");
    return false;
  }
  ROS_INFO("[FilterChain] Bandpass filter configuration succeeded");

  // ...
  // other implementation
  // ...
  
  // Dump filter configuration
  butter_bp.dump();
  
  // Apply the filte
  outbp = butter_bp.apply(input);

  ros::shutdown();
	
  return 0;
}
```
And the corresponding YAML configuration file:
```
ButterworthBandPass:
    - name: butterworth-lowpass
      type: rosneuro_filters/ButterworthFilterDouble
      params: 
        samplerate: 512
        type: lowpass
        order: 4
        cutoff: 10
    - name: butterworth-highpass
      type: rosneuro_filters/ButterworthFilterDouble
      params: 
        samplerate: 512
        type: highpass
        order: 4
        cutoff: 1
```


  
