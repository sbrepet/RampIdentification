# RampIdentification
 Uncertainties of Ramp Identification in 100 Equidistant Noisy Data Points

Date: 20Sep2023
Author: Sebastian Repetzki
Kind of Project: Research project in statistics and signal processing.
Project report: see Preprint
Code language: C++

Description: A real-time process that provides data in regular intervals 
			is expected to show a constant, though noisy signal that, 
			at some unpredictable point in time, starts growing linearly at a non-predictable rate,
			while still suffering from constant gaussian noise.
			This code, "Uncertainties of Ramp Identification", finds best estimates for
				time increment where the ramp started
				inclination of the ramp
				actual height of the ramp 
				standard deviation of the signal noise
			Even more importantly, the code estimates the uncertainties of all the four estimates above:
				Uncertainty of the ramp start estimate
				Uncertainty of the inclination estimate
				Uncertainty of the actual height estimate
				Uncertainty of the signal noise estimate
			All Uncertainty estimates are given with a user-chosen confidence level of either >99,73% or >99,99%.
			
			
			
