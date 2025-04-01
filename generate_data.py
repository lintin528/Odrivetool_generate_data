import odrive
from odrive.enums import *
import time
import csv
import random
import numpy as np

# Find a connected ODrive (this will block until you connect one)
print("Finding an ODrive...")
odrv0 = odrive.find_any()
if odrv0:
    print(f"find motor ")

def sine_wave_input(time, frequency=5, amplitude=1):
    """ generate (Sine Wave) input signal """
    return amplitude * np.sin(2 * np.pi * frequency * time)

def three_stage_ramp(time, t_peak1, t_peak2, slope_up1=1, slope_down=-1, slope_up2=0.5):
    """ generate (Ramp) input signal """
    y = np.piecewise(time,
        [time < t_peak1, 
         (time >= t_peak1) & (time < t_peak2), 
         time >= t_peak2],
        [lambda t: slope_up1 * t,  
         lambda t: slope_down * (t - t_peak1) + slope_up1 * t_peak1,  
         lambda t: slope_up2 * (t - t_peak2) + (slope_down * (t_peak2 - t_peak1) + slope_up1 * t_peak1)]
    )
    
    return y

get_input = {
    'sine': sine_wave_input,
    'three_stage_ramp': three_stage_ramp,
}

# Open a CSV file to write the data
iteration_times = 5
Ts = 0.1
time_array = np.arange(0, 12, Ts)

for i in range(iteration_times):
    freq = np.random.uniform(0.5, 1)
    sine_amplitude = np.random.uniform(1, 2)

    t_peak1 = np.random.uniform(2, 5)
    t_peak2 = np.random.uniform(t_peak1 + 4, 6)
    slope_up1 = np.random.uniform(1, 2)
    slope_down = np.random.uniform(-2, -0.5)
    print(slope_down, t_peak1, t_peak2)
    slope_up2 = np.random.uniform(0.5, 3)

    with open(f'sine_input_{i}.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Timestamp', 'Encoder Position', 'target position'])

        start_time = time.time()
        start_pos, position  = odrv0.axis0.pos_estimate, odrv0.axis0.pos_estimate

        odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        odrv0.axis0.controller.input_pos = position

        target_positions = get_input['sine'](time_array, frequency=freq, amplitude=sine_amplitude)
        for j in range(len(target_positions)):
            timestamp = time.time() - start_time

            position = start_pos + target_positions[j]
            odrv0.axis0.controller.input_pos = position
            encoder_pos = odrv0.axis0.pos_estimate

            writer.writerow([timestamp, (encoder_pos-start_pos), (position-start_pos)])
            time.sleep(0.1)  # Adjust the sleep time as needed
    print(f"run step input round {i} \n")

    with open(f'three_stage_ramp_input_{i}.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Timestamp', 'Encoder Position', 'target position'])
        
        # Collect data for a certain period
        start_time = time.time()
        start_pos, position  = odrv0.axis0.pos_estimate, odrv0.axis0.pos_estimate

        odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        odrv0.axis0.controller.input_pos = position

        target_positions = get_input['three_stage_ramp'](time_array, t_peak1=t_peak1, t_peak2=t_peak2, slope_up1=slope_up1, slope_down=slope_down, slope_up2=slope_up2)
        print(target_positions)
        for j in range(len(target_positions)):
            timestamp = time.time() - start_time

            position = start_pos + target_positions[j]
            odrv0.axis0.controller.input_pos = position
            encoder_pos = odrv0.axis0.pos_estimate

            writer.writerow([timestamp, (encoder_pos-start_pos), (position-start_pos)])
            time.sleep(0.1)  # Adjust the sleep time as needed
    print(f"run ramp input round {i} \n")

print("Data collection complete.")
