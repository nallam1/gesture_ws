import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import ADS1263
import RPi.GPIO as GPIO

import numpy as np
from scipy.signal import butter, firwin, lfilter, iirnotch
from statsmodels.tsa.ar_model import AutoReg

# Design filters globally or in __init__
# Bandpass FIR (20–500 Hz, fs=1000 Hz, 36 taps)
fs = 1000  # Sampling rate (adjust if needed)
num_taps = 36
bandpass_taps = firwin(num_taps, [20, 500], pass_zero=False, fs=fs)

# Notch filter at 60 Hz
notch_freq = 60.0
q = 30.0  # Quality factor
b_notch, a_notch = iirnotch(notch_freq, q, fs)


class SEMGReaderNode(Node):
    def __init__(self):
        super().__init__('semg_reader_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'semg_data', 10)
        timer_period = 0.0011/fs  # 1000 Hz sampling (1 ms)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize ADC
        self.adc = ADS1263.ADS1263()
        if self.adc.ADS1263_init_ADC1('ADS1263_GAIN_1', 'ADS1263_10SPS') == -1:
            self.get_logger().error('Failed to initialize ADS1263')
            exit()

        self.adc.ADS1263_SetMode(1)  # Single-ended
        self.channel_list = [0, 1, 2, 3]  # Channels for 4 sEMG electrodes

        self.get_logger().info('sEMG Reader Node started and ADC initialized.')

    def timer_callback(self):
        # Read values from 4 sEMG channels
        try:
            adc_values = self.adc.ADS1263_GetAll()
            selected_values = [adc_values[i] for i in self.channel_list]
            # Convert to microvolts (assumes default reference voltage of 5.0V)
            converted = [(val * 5.0 / 0x7fffffff) * 1e6 for val in selected_values]
	    filtered_signals = [filter_and_rectify(np.array([val])) for val in converted]

		def filter_and_rectify(converted):
		    # Apply bandpass FIR filter
		    bandpassed = lfilter(bandpass_taps, 1.0, signal)
		    
		    # Apply notch filter
		    notched = lfilter(b_notch, a_notch, bandpassed)
		    
		    # Rectification
		    rectified = np.abs(notched)
		    
		    return rectified

		# Feature extraction function
		def extract_time_domain_features(signal, threshold=0.01):
		    features = {}
		    features['MAV'] = np.mean(np.abs(signal)) #Moving time average
		    features['RMS'] = np.sqrt(np.mean(signal**2)) # Moving Root mean square
		    features['WL'] = np.sum(np.abs(np.diff(signal)))  #Waveform Length 
		    zc = np.where(np.diff(np.signbit(signal)))[0]
		    features['ZC'] = len(zc) #Zero-crossings
		    diff1 = np.diff(signal)
		    diff2 = np.diff(diff1)
		    ssc = np.sum(((diff1[:-1] * diff1[1:]) < 0) & (np.abs(diff2) >= threshold))
		    features['SSC'] = ssc #Slope Sign Change
		    amp_diffs = np.abs(np.diff(signal))
		    features['AR'] = compute_ar_coefficients(signal, order=4)
			def compute_ar_coefficients(signal, order=4):
			    """
			    Compute AR (auto-regression) coefficients for a single sEMG signal segment.

			    Args:
			        signal (np.array): 1D array of sEMG signal for one window.
			        order (int): Order of the AR model.

			    Returns:
			        np.array: Array of AR coefficients of length `order`.
			    """
			    # Fit the AR model
			    model = AutoReg(signal, lags=order, old_names=False)
			    model_fit = model.fit()

			    # Extract AR coefficients (excluding the intercept)
				ar_coefficients = model_fit.params[1:]  # Skip the intercept term

				return ar_coefficients
		    
		   #features['WAMP'] = np.sum(amp_diffs > threshold) #Willison amplitude, number of times pass some reference voltage
		    return features

	    # For each filtered signal (1 per channel), extract features
	    features_per_channel = [
	    extract_time_domain_features(ch_signal)
		 for ch_signal in filtered_signals
            ]

	    # Flatten to one vector: [CH0_MAV, CH0_RMS, ..., CH3_WAMP]
	    flat_feature_vector = []
	    for ch_features in features_per_channel:
	        flat_feature_vector.extend(ch_features.values())

	    # Now publish as Float32MultiArray
	    msg = Float32MultiArray()
	    msg.data = flat_feature_vector
	    self.publisher_.publish(msg)

	    self.get_logger().debug(f'Published sEMG data: {flat_feature_vector}')

        except Exception as e:
            self.get_logger().error(f'ADC read failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SEMGReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.adc.ADS1263_Exit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
