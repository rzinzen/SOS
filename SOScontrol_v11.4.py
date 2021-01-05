#written and tested for python3

#####################################################################################################
# Filename      :   SOScontrol_v11.4_optimizingParameters.py
# Description   :   Automating the SOS
# Author        :   Zinzen
# modification  :   2021/01/03
#####################################################################################################

#NOTE: this Python Code is thouroughly commented to make it as understandable as possible.

#####################################################################################################
#####################################################################################################




#####################################################################################################
#====================================================================================================
#==============================                                        ==============================
#==============================            IMPORT LIBRARIES            ==============================
#==============================                                        ==============================
#====================================================================================================
#####################################################################################################

import RPi.GPIO as GPIO
import time
import datetime
from ADCDevice import *



#####################################################################################################
#====================================================================================================
#==============================                                        ==============================
#==============================     DEFINITION OF GLOBAL VARIABLES     ==============================
#==============================                                        ==============================
#====================================================================================================
#####################################################################################################


###MOTOR VARIABLES
    # NOTE: motor turn direction will be determined by which pin is high, and which is low,
            #  they output to the MCC's (motor control chip's) INput pins,
            #  --> this will toggle motor voltage on the corresponding MCC OUTput pins.  
            # if both polarity pins are low or high, then there is no Voltage b/w the pins => no movement

#YAW MOTOR variables for connections on the GPIO
yaw_motor_pin1 = 13     # this pin is for motor polarity = GpIO27
yaw_motor_pin2 = 15     # this pin is for motor polarity = GpIO22
yaw_enable_pin = 11     # this pin is for PWM signal     = GpIO17

# PITCH MOTOR varioables for connections on the GPIO
pitch_motor_pin1 = 18   # this pin is for motor polarity = GpIO24
pitch_motor_pin2 = 22   # this pin is for motor polarity = GpIO25
pitch_enable_pin = 16   # this pin is for PWM signal     = GpIO23


### ADC variables
adc = ADCDevice()   # Define an ADCDevice class object
V_out = 3.3         # sets conversion factor to compute voltage from ADC, should be the voltage taken from the voltage devider
volt_generated = 0  # sets initial variable value for measured voltage at solar cell


### movement parameters
direction = 1          # set initial movement direction (1 = FORward, 0 = STOP, -1 = BACKward)

move_time = 0          # set move_time variable. NOTE: move_time is directional (+/-)

pos_yaw   = 0          # sets initial yaw   position (mesured in time)   
pos_pitch = 0          # sets initial pitch position (mesured in time)

yaw_dc      = 100      # sets duty cycle (dc) for yaw motor 
pitch_dc    = 100      # sets duty cycle (dc) for pitch motor 

t_rev_yaw   = 18.63    # time it takes to revolve 1 time in yaw (around)
t_rev_pitch = 127.44   # time it takes to revolve 1 time in pitch (head-over-head)

degree_coarse       = 15            # degrees by which coarse adjustments will be made
degree_fine         = 5             # degrees by which fine adjustments will be made
degree_coarse_yaw   = degree_coarse # degrees by which yaw coarse adjustments will be made
degree_fine_yaw     = degree_fine   # degrees by which yaw fine adjustments will be made
degree_coarse_pitch = degree_coarse # degrees by which pitch coarse adjustments will be made
degree_fine_pitch   = degree_fine   # degrees by which pitch fine adjustments will be made

adj_coarse_yaw   = (degree_coarse_yaw   * (t_rev_yaw / 360))     # calculation to convert degrees of movement into time of motor_ON to allow travel by that many degrees
adj_coarse_pitch = (degree_coarse_pitch * (t_rev_pitch / 360))   # ""
adj_fine_yaw     = (degree_fine_yaw     * (t_rev_yaw / 360))     # ""
adj_fine_pitch   = (degree_fine_pitch   * (t_rev_pitch / 360))   # ""

coarse_or_fine = 'coarse' # define the adjustment grain variable as initially coarse
                              # possible values are (coarse, fine, none)
                                  # ('coarse') --> triggers adjustment by (degree_coarse)
                                  # ('fine')   --> triggers adjustment by (degree_fine)
                                  # ('none')   --> triggers loop exit or pause

rest_interval_min = 1
rest_interval_sec = rest_interval_min * 60       #rest interval between re-adjustments, in minutes converted to seconds





####################################################################################################
#====================================================================================================
#==============================                                        ==============================
#==============================       DEFINING MODULAR FUNCTIONS
#==============================
#==============================                                        ==============================
#====================================================================================================
#####################################################################################################


def setup():          # inial setup: checks for adc, configures GPIO pis, creates PWMs, DEFINES COMMON PARAMETERS 
        
    print('\n','\n','\n','[setup() function has been called -- now running basic set-up] ...)','\n')

    ### ADC SET-UP
        #(ADC = analog-digital converter)
        #NOTE: we are using the ADC 'ADS7830', which is an SI-bus enabled ADC. 
        #The ADS7830 has 8-bit resolution, which means it has 2^8 = 256 levels
    global adc  #makes the adc available globally
    if(adc.detectI2C(0x4b)):    # Detect the ADS7830 (ADC)
        adc = ADS7830()  
        print('The correct ADC (analog-to-digital converter) device was connencted:',adc)
    else:
        print("Correct I2C address was not found, \n"
            "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
            "Program Exit. \n");
        exit(-1)

    ### INITIAL GPIO SET-UP
    GPIO.setwarnings(False)
    
    GPIO.setmode(GPIO.BOARD)                  # use physical GPIO numbering
    
    GPIO.setup(yaw_motor_pin1, GPIO.OUT)      # set pin to OUTPUT mode
    GPIO.output(yaw_motor_pin1, GPIO.LOW)     # set initial pin output to LOW
    GPIO.setup(yaw_motor_pin2, GPIO.OUT)      # set pin to OUTPUT mode
    GPIO.output(yaw_motor_pin2, GPIO.LOW)     # set initial pin output to LOW
    GPIO.setup(yaw_enable_pin, GPIO.OUT)      # set pin to OUTPUT mode

    GPIO.setup(pitch_motor_pin1, GPIO.OUT)    # set pin to OUTPUT mode
    GPIO.output(pitch_motor_pin1, GPIO.LOW)   # set initial pin output to LOW
    GPIO.setup(pitch_motor_pin2, GPIO.OUT)    # set pin to OUTPUT mode
    GPIO.output(pitch_motor_pin2, GPIO.LOW)   # set initial pin output to LOW
    GPIO.setup(pitch_enable_pin, GPIO.OUT)    # set pin to OUTPUT mode

    ### PWM setup for movement speed:
        # the motor speeds can be controlled by giving a Pulse-width modulation (PWM) signal to the enable pins
        #in PWM, the duty cycle (dc) dictates the percentage of time per cycle power is ON (dc=100 -> motor operates at 100% power)

    global pwm_yaw                              # makes the variable available globally 
    pwm_yaw = GPIO.PWM(yaw_enable_pin,100)      # creats PWM at a frequency of 0.1 kHz (100 cycles / second)
    pwm_yaw.start(0)                            # starts PWM with an initial dc of 

    global pwm_pitch                            # makes the variable available globally  
    pwm_pitch = GPIO.PWM(pitch_enable_pin,100)  # creats PWM at a frequency of 0.1 kHz (100 cycles / second)
    pwm_pitch.start(0)                          # starts PWM with an initial dc of 0
    
    ### initial setting of recurring variables
    global direction
    direction = 1
    global move_time
    move_time = 0
    global pos_yaw
    pos_yaw = 0
    global pos_pitch
    pos_pitch = 0
    global volt_generated
    volt_generated = 0


#====================================================================================================

def import_pos_val_opt():

    print('\n','\n','\n','[import_pos_val_opt() function has been called -- now giving option to import positional values for positional re-set] ...)','\n')

    choosing = True
    while (choosing == True):

        choice = input('\n''Would you like to import previously stored positional values to help reset the SOS to origin? \n'
        '(if No, the SOS will use its current position in yaw and pitch as origin) \n'
        '(y / n ) :  ')

        if (choice == 'y'):
            print ('\n''You have chosen YES')
            choosing = False
            try:
                get_saved_pos_val()
                reset_pos()
            except IOError:
                print('\n'
                      '§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n'
                      '\n'
                      'No "position_save.txt" file was found in the same directory as the script.\n'
                      '  -->  Please assure the file is placed into the directory and \n'
                      '       then press "y", or choose "n"')
                choosing = True
        elif (choice == 'n'):
            print ('\n''you have chosen NO --> the current position will be taken as origin')
            choosing = False
        else :
            print('\n'
                  '§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n'
                  '\n'
                  'please answer "y" for yes, or "n" for NO ...')

    print('exited with choice', choice)


#====================================================================================================


def measure_volt():          # measures value on adc and defines variables (avg_adc_value, volt_on_adc, volt_generated)
            
    print('\n','\n','\n','[measure_volt() function has been called -- now measuring voltage] ...)','\n')
    
    cum_adc_value = 0                               # initially sets variable to 0
    iterations = 10                                 # number of iterations to be averaged across
    iteration_interval = 0.2                        # time between iterative measurements 

    print('Averaging voltage measurements in current position...')
    for i in range(0, iterations, 1):               # conduct 10 consecutive reads
        adc_value = adc.analogRead(0)               # reads the (0-255) value at adc channel 0
        cum_adc_value = cum_adc_value + adc_value   # read and sum the ADC value of channel 0 over the iteration
        print('measurement', i+1, ': the mesured ADC Value is', adc_value)
        time.sleep(iteration_interval)                              # time between measurements
    
    avg_adc_value = cum_adc_value / iterations      # calculates the average ADC value
    print('The average ADC value in this position is', avg_adc_value)
    
    volt_on_adc = avg_adc_value / 255.0 * V_out     # calculate the approximate actual voltage on the 8-bit ADC (after the voltage splitter)
    
    global volt_generated
    volt_generated = volt_on_adc * 5.6              # calculate the approximate actual voltage generated by the solar cell (w/o the voltage splitter) 
                                                            # the conversion factor for a 10+46 k Ohm V-splitter is 5.6)
    print ('This computes to: \n'
               '~', round(volt_on_adc, 2) ,'V on the ADC post V-splitter; therefore \n'
               '~', round(volt_generated, 2), 'V is generated in this position by the solar cell')


#====================================================================================================


def report_pos():
    global pos_deg_pitch
    global pos_deg_yaw
        
    print('\n','\n','\n','[report_pos() function has been called] ...)','\n')

    print ('The SOS is currently in the following position (default origin is 0°, 0°): \n'
            '   YAW   position (device rotation) currently is ',(round(pos_yaw, 2)),'=',(round(pos_yaw*360/t_rev_yaw, 2)),'° \n'
            '   PITCH position (cell   rotation) currently is ',(round(pos_pitch, 2)),'=',(round(pos_pitch*360/t_rev_pitch, 2)),'°')


#====================================================================================================


def yaw_move():          # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
        
    print('\n','\n','\n','[yaw_move() function has been called -- now moving YAW] ...)','\n')
    
    global move_time
    global direction
    global pos_yaw

    pwm_yaw.ChangeDutyCycle(yaw_dc)                 # change PMW (duty cycle)
    print ('PWM duty cycle for Yaw motors is', yaw_dc, '%')

    if (direction > 0):                             # make motor turn forward if direction = 1
        GPIO.output(yaw_motor_pin1,GPIO.HIGH)       # motoRPin1 output HIGH level
        GPIO.output(yaw_motor_pin2,GPIO.LOW)        # motoRPin2 output LOW level
        print ('Turning yaw forward...')
    elif (direction < 0):                           # make motor turn forward if direction = -1
        GPIO.output(yaw_motor_pin1,GPIO.LOW)        # motoRPin1 output LOW level
        GPIO.output(yaw_motor_pin2,GPIO.HIGH)       # motoRPin2 output HIGH level# make motor turn backward
        print ('Turning yaw backward...')
    else :                                          # stop motor if direction = 0
        GPIO.output(yaw_motor_pin1,GPIO.LOW)        # motoRPin1 output LOW level
        GPIO.output(yaw_motor_pin2,GPIO.LOW)        # motoRPin2 output LOW level
        print ('No direction -> Motor stopped...')
    
    time.sleep(abs(move_time))                      # allows movement time
    
    GPIO.output(yaw_motor_pin1,GPIO.LOW)            # motoRPin1 output LOW level
    GPIO.output(yaw_motor_pin2,GPIO.LOW)            # motoRPin2 output LOW level    
    
    pwm_yaw.ChangeDutyCycle(0)                      # set duty cycle

    pos_yaw = pos_yaw + (move_time * direction)     # redefines stored position by adding directional (+/-) movement time
    

#====================================================================================================


def pitch_move():          # moves pitch motor by variables (direction, move_time), then re-defines pos_pitch
        
    print('\n','\n','\n','[pitch_move() function has been called -- now moving PITCH] ...)','\n')
    
    global move_time
    global direction
    global pos_pitch
       
    pwm_pitch.ChangeDutyCycle(pitch_dc)             # change PMW (duty cycle)
    print ('PWM duty cycle for Pitch motor is', pitch_dc, '%')

    if (direction > 0):                             # make motor turn forward if direction = 1
        GPIO.output(pitch_motor_pin1,GPIO.HIGH)     # motoRPin1 output HIGH level
        GPIO.output(pitch_motor_pin2,GPIO.LOW)      # motoRPin2 output LOW level
        print ('Turning pitch forward...')
    elif (direction < 0):                           # make motor turn forward if direction = -1
        GPIO.output(pitch_motor_pin1,GPIO.LOW)      # motoRPin1 output LOW level
        GPIO.output(pitch_motor_pin2,GPIO.HIGH)     # motoRPin2 output HIGH level# make motor turn backward
        print ('Turning pitch backward...')
    else :                                          # stop motor if direction = 0
        GPIO.output(pitch_motor_pin1,GPIO.LOW)      # motoRPin1 output LOW level
        GPIO.output(pitch_motor_pin2,GPIO.LOW)      # motoRPin2 output LOW level
        print ('Motor stopped...')

    time.sleep(abs(move_time))                      # allows movement time

    GPIO.output(pitch_motor_pin1,GPIO.LOW)          # motoRPin1 output LOW level
    GPIO.output(pitch_motor_pin2,GPIO.LOW)          # motoRPin2 output LOW level    
    
    pwm_pitch.ChangeDutyCycle(0)                    # set duty cycle

    pos_pitch = pos_pitch + (move_time * direction) # redefines stored position by adding directional (+/-) movement time
    
   
    
#====================================================================================================


def pitch_tilt_45():            # movement of pitch by 45°
    
    print('\n','\n','\n','[pitch_tilt_45() function has been called -- now tilting pitch by ~ +45°] ...)','\n')
    
    global direction
    direction = 1
    
    global move_time
    move_time= t_rev_pitch / 8
    
    pitch_move() 
    
   
#====================================================================================================


def yaw_sample_120():          # sample voltages at 3 locations rotated by 120°
        
    global direction
    global move_time
    move120_in_sec = round((t_rev_yaw / 3), 2)
    
    print('\n','\n','\n','[yaw_sample() function has been called -- now sampling data at 3 positions offset by 120°] ...)','\n')
        
    ### voltage at origin
    measure_volt()                      # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
    volt_at_ori = volt_generated           # store origin voltage

    ### voltage at +120°
    direction = 1
    move_time = move120_in_sec
    print('Moving to +120° yaw...')
    yaw_move()                          # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
    measure_volt()                      # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
    volt_at_120 = volt_generated        # store voltage 120° from yaw origin
    
    ### voltage at -120°
    direction = -1
    move_time = 2 * move120_in_sec
    print('Moving to -120° yaw')
    yaw_move()                          # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
    measure_volt()                      # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
    volt_at_240 = volt_generated        # store voltage 240° from yaw origin
    
    ### evaluating voltages
    if (volt_at_240 > volt_at_120) and (volt_at_240 > volt_at_ori) :    # if highest voltage measured at 240
        print('starting to optimize at yaw position -120° from origin')
    elif (volt_at_120 > volt_at_240) and (volt_at_120 > volt_at_ori) :  # if highest voltage measured at 120
        direction = 1
        move_time = 2 * move120_in_sec
        yaw_move()                          # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
        print('starting to optimize at yaw position 120° from origin')
    elif (volt_at_ori > volt_at_240) and (volt_at_ori > volt_at_120) :  # if highest voltage measured at origin
        direction = 1
        move_time = move120_in_sec
        yaw_move()                          # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
        print('starting to optimize at yaw origin')
            # the following is in case 2 voltage measurements are equal
    elif (volt_at_ori > volt_at_240) :
        direction = 1
        move_time = move120_in_sec
        yaw_move()                          # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
        print('starting to optimize at yaw origin')
    elif (volt_at_120 > volt_at_240) :
        direction = 1
        move_time = 2 * move120_in_sec
        yaw_move()                          # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
        print('starting to optimize at yaw position 120° from origin')
    else:
        print('starting to optimize at yaw position -120° from origin')


#====================================================================================================


def yaw_adj():
    
    print('\n','\n','\n','[yaw_adj() function has been called -- now beginning coarse and fine YAW optimization] ...)','\n')
    
    global direction
    global move_time
    
    
    ### YAW COARSE ADJUSTMENT LOOP
    
    print('commencing YAW optimization (first coarse, then fine)')
        
    coarse_or_fine = 'coarse'
    
    direction = 1
    dir_t_0 = direction
    dir_t_1 = 0.3                   # 0.3 is simply a dummy-value that cannot sum to 0
    dir_t_2 = 0.3
    dir_t_3 = 0.3
    
    measure_volt()                  # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
    volt_t_0 = volt_generated       # t_0 is the most recent measurement
       
    while (coarse_or_fine == 'coarse'):
        move_time = abs(adj_coarse_yaw)
        yaw_move()                      # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
        
        measure_volt()                  # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
        
        # the following series passes the volt_generated value backward, t_0 being the most recent measurement
        volt_t_1 = volt_t_0             # t_1 is the measurement from 1 round ago
        volt_t_0 = volt_generated       # t_0 is the most recent measurement

        if (volt_t_0 < volt_t_1):
            direction = direction * -1  # inverse direction parameter
            print('voltage in new yaw position is lower --> inversing yaw adjustment direction')
        else :
            print('voltage in new yaw position is higher --> maintaining yaw adjustment direction')
        
        # the following series passes the direction value backward, t_0 being the most recent directional assignment for the next movement
        dir_t_3 = dir_t_2               # t_3 is the directional assignment from 3rd-to-last rotation
        dir_t_2 = dir_t_1               # t_2 is the directional assignment from 2nd-to-last rotation
        dir_t_1 = dir_t_0               # t_1 is the directional assignment from the last rotation
        dir_t_0 = direction

        if (dir_t_0 + dir_t_1 + dir_t_2 + dir_t_3 == 0):    # the pattern of the last 3 rotational directions (+1 or -1) plus the next one 
                                                            # summing up to 0 is expected if the next move brings the SOS back to the maximum voltage yaw position
            yaw_move()                  # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
            print('coarse YAW optimization found a rough maximum --> proceeding to fine adjustment')
            coarse_or_fine = 'fine'

  
    ### YAW FINE ADJUSTMENT LOOP

    print('commencing fine YAW optimization')
        
    direction = 1
    dir_t_0 = direction
    dir_t_1 = 0.3                   # 0.3 is simply a dummy-value that cannot sum to 0
    dir_t_2 = 0.3
    dir_t_3 = 0.3
    
    while coarse_or_fine == 'fine':
        move_time = abs(adj_fine_yaw)
        yaw_move()                      # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
        
        measure_volt()                  # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
      
        # the following series passes the volt_generated value backward, t_0 being the most recent measurement
        volt_t_1 = volt_t_0             # t_1 is the measurement from 1 round ago
        volt_t_0 = volt_generated       # t_0 is the most recent measurement

        if (volt_t_0 < volt_t_1):
            direction = direction * -1  # inverse direction parameter
            print('voltage in new yaw position is lower --> inversing yaw adjustment direction')
        else:
            print('voltage in new yaw position is higher --> maintaining yaw adjustment direction')

        # the following series passes the direction value backward, t_0 being the most recent directional assignment for the next movement
        dir_t_3 = dir_t_2               # t_3 is the directional assignment from 3rd-to-last rotation
        dir_t_2 = dir_t_1               # t_2 is the directional assignment from 2nd-to-last rotation
        dir_t_1 = dir_t_0               # t_1 is the directional assignment from the last rotation
        dir_t_0 = direction

        if (dir_t_0 + dir_t_1 + dir_t_2 + dir_t_3 == 0):    # the pattern of the last 3 rotational directions (+1 or -1) plus the next one 
                                                            # summing to 0 is expected if the next move brings the SOS back to the maximum voltage yaw position
            yaw_move()                  # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
            print('fine YAW rotation found a maximum --> proceeding to pitch adjustment')
            coarse_or_fine = 'none'
            

#====================================================================================================


def pitch_adj():
    
    print('\n','\n','\n','[pitch_adj() function has been called -- now beginning coarse and fine PITCH optimization] ...)','\n')
    
    global direction
    global move_time
    
    
    ### PITCH COARSE ADJUSTMENT LOOP
    
    print('commencing YAW optimization (first coarse, then fine)')
        
    coarse_or_fine = 'coarse'
    
    direction = 1
    dir_t_0 = direction
    dir_t_1 = 0.3                   # 0.3 is simply a dummy-value that cannot sum to 0
    dir_t_2 = 0.3
    dir_t_3 = 0.3
    
    measure_volt()                  # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
    volt_t_0 = volt_generated       # t_0 is the most recent measurement
       
    while (coarse_or_fine == 'coarse'):
        move_time = abs(adj_coarse_pitch)
        pitch_move()                      # moves pitch motor by variables (direction, move_time), then re-defines global pos_pitch
        
        measure_volt()                  # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
        
        # the following series passes the volt_generated value backward, t_0 being the most recent measurement
        volt_t_1 = volt_t_0             # t_1 is the measurement from 1 round ago
        volt_t_0 = volt_generated       # t_0 is the most recent measurement

        if (volt_t_0 < volt_t_1):
            direction = direction * -1  # inverse direction parameter
            print('voltage in new pitch position is lower --> inversing pitch adjustment direction')
        else :
            print('voltage in new pitch position is higher --> maintaining pitch adjustment direction')
        
        # the following series passes the direction value backward, t_0 being the most recent directional assignment for the next movement
        dir_t_3 = dir_t_2               # t_3 is the directional assignment from 3rd-to-last rotation
        dir_t_2 = dir_t_1               # t_2 is the directional assignment from 2nd-to-last rotation
        dir_t_1 = dir_t_0               # t_1 is the directional assignment from the last rotation
        dir_t_0 = direction

        if (dir_t_0 + dir_t_1 + dir_t_2 + dir_t_3 == 0):    # the pattern of the last 3 rotational directions (+1 or -1) plus the next one 
                                                            # summing up to 0 is expected if the next move brings the SOS back to the maximum voltage yaw position
            pitch_move()                  # moves pitch motor by variables (direction, move_time), then re-defines global pos_pitch
            print('coarse PITCH optimization found a rough maximum --> proceeding to fine adjustment')
            coarse_or_fine = 'fine'

  
    ### YAW FINE ADJUSTMENT LOOP

    print('commencing fine PITCH optimization')
        
    direction = 1
    dir_t_0 = direction
    dir_t_1 = 0.3                   # 0.3 is simply a dummy-value that cannot sum to 0
    dir_t_2 = 0.3
    dir_t_3 = 0.3
    
    while coarse_or_fine == 'fine':
        move_time = abs(adj_fine_pitch)
        pitch_move()                      # moves pitchyaw motor by variables (direction, move_time), then re-defines global pos_pitch
        
        measure_volt()                  # measures value on adc and defines variables (avg_adc_value, volt_on_adc, global volt_generated)
      
        # the following series passes the volt_generated value backward, t_0 being the most recent measurement
        volt_t_1 = volt_t_0             # t_1 is the measurement from 1 round ago
        volt_t_0 = volt_generated       # t_0 is the most recent measurement

        if (volt_t_0 < volt_t_1):
            direction = direction * -1  # inverse direction parameter
            print('voltage in new pitch position is lower --> inversing pitch adjustment direction')
        else:
            print('voltage in new pitch position is higher --> maintaining pitch adjustment direction')

        # the following series passes the direction value backward, t_0 being the most recent directional assignment for the next movement
        dir_t_3 = dir_t_2               # t_3 is the directional assignment from 3rd-to-last rotation
        dir_t_2 = dir_t_1               # t_2 is the directional assignment from 2nd-to-last rotation
        dir_t_1 = dir_t_0               # t_1 is the directional assignment from the last rotation
        dir_t_0 = direction

        if (dir_t_0 + dir_t_1 + dir_t_2 + dir_t_3 == 0):    # the pattern of the last 3 rotational directions (+1 or -1) plus the next one 
                                                            # summing to 0 is expected if the next move brings the SOS back to the maximum voltage yaw position
            pitch_move()                  # moves pitch motor by variables (direction, move_time), then re-defines global pitch
            print('fine pitch rotation found a maximum --> YAW and PITCH are now OPTIMIZED')
            coarse_or_fine = 'none'
            

#====================================================================================================


def reset_pos():          # returns SOS to default origin
            
    print('\n','\n','\n','[reset_pos() function has been called -- now resetting to origin] ...)','\n')
    
    global direction
    global move_time
    global pos_yaw
    global pos_pitch

    # return yaw to origin
    if (pos_yaw > 0):
        direction = -1
        print('going backward to return SOS to yaw origin...')
    elif (pos_yaw < 0):
        direction = 1
        print('going forward to return SOS to yaw origin...')
    else :
        print('SOS is already at yaw origin - no yaw movement required.')

    move_time = abs(pos_yaw)
    
    yaw_move()
    
    pos_yaw = 0
    
    
    # return pitch to origin
    if (pos_pitch > 0):
        direction = -1
        print('going backward to return SOS to pitch origin...')
    elif (pos_pitch < 0):
        direction = 1
        print('going forward to return SOS to pitch origin...')
    else :
        print('SOS is already at pitch origin - no pitch movement required.')

    move_time = abs(pos_pitch)
    
    pitch_move()
    
    pos_pitch = 0
    

#====================================================================================================


def save_pos_val():          # save positional values to file
    
    print('\n','\n','\n','[save_pos_val() function has been called -- now initiating saving positional values to file] ...)','\n')
    
    report_pos()
    
    position_save_file = open("position_save.txt", "w")
    position_save_file.write(
        "pos_yaw = " + "\n"
        + str(pos_yaw) + "\n"
        + "pos_pitch = " + "\n"
        + str(pos_pitch) + "\n")
    position_save_file.close()
    print('The current positional values (pos_yaw, pos_pitch) have been saved to file. /n'
            '   Note: The save directory is the same as where the script is saved. /n'
            '         The filename is "position_save.txt".')

    
#====================================================================================================


def get_saved_pos_val():           # get positional values saved to file
    
    print('positional values BEFORE value retrieval:')
    report_pos()

    saved_positions = open('position_save.txt', 'r')
    line1 = saved_positions.readline()
    line2 = saved_positions.readline()
    line3 = saved_positions.readline()
    line4 = saved_positions.readline()
    global pos_yaw
    pos_yaw = float(line2)
    global pos_pitch
    pos_pitch = float(line4)
    print('pos_yaw set to save yaw position: pos_yaw =', pos_yaw)
    print('pos_pitch set to save pitch position: pos_pitch =', pos_pitch)
    
    print('positional values AFTER value retrieval:')

    report_pos()



#====================================================================================================


def destroy():
        
    print('\n','\n','\n','[destroy() function has been called -- now initiating clean-up] ...)','\n')
    
    save_pos_val()     
   
    GPIOIO.output(yaw_motor_pin1,GPIO.LOW)		# motoRPin1 output LOW level
    GPIO.output(yaw_motor_pin2,GPIO.LOW)        # motoRPin2 output LOW level
    GPIO.output(pitch_motor_pin1,GPIO.LOW)		# motoRPin1 output LOW level
    GPIO.output(pitch_motor_pin2,GPIO.LOW)		# motoRPin2 output LOW level
        
    pwm_yaw.stop()								# stopping yaw PWM
    pwm_pitch.stop()							# stopping pitch PWM
    GPIO.cleanup()								# Release all GPIO assignments

    print('...\n','...\n','PROGRAM HAS ENDED!','\n','\n')


#====================================================================================================


def workflow():

    # available functions:

	# setup()             # inial setup: checks for adc, configures GPIO pis, creates PWMs, DEFINES COMMON PARAMETERS
        # import_pos_val_opt()# allows for import of saved positional values from file
	# save_pos_val()      # save positional values to file
	# get_saved_pos_val() # get positional values saved to file
	# measure_volt()      # measures value on adc and defines variables (avg_adc_value, volt_on_adc, volt_generated)
	# report_pos()		      # reports currently recorded position
	# yaw_move()		      # moves yaw motor by variables (direction, move_time), then re-defines global pos_yaw
	# pitch_move()	    	  # moves pitch motor by variables (direction, move_time), then re-defines pos_pitch
	# pitch_tilt_45()		   # movement of pitch by 45°
	# yaw_sample_120()	   # sample voltages at 3 locations rotated by 120°
	# yaw_adj()		      # coarse and then fine adjust for YAW
	# pitch_adj()		      # coarse and then fine adjust for PITCH
	# reset_pos()		      # returns SOS to default origin
	# destroy()		      # report_pos, reset_pos, report_pos, GPIO cleanup, stop PWMs

    setup()
    
    import_pos_val_opt()

    print('\n','Taking and printing 2 consecutive voltage test measurment sets \n'
             '(alter light conditions on solar cell (e.g. shade) to see if measurements change) ...')  
    measure_volt()
    time.sleep(5)
    measure_volt()
    
    
    pitch_tilt_45()
        
    yaw_sample_120()

    report_pos()
        
    print('ENTERING ADJUSTMENT LOOP ...')
    loop_count = 1

    while True:
        print('commencing adjustment loop iteration', loop_count)
        
        yaw_adj()
 
        pitch_adj()
    
        loop_count = loop_count + 1

        print('\n','\n','(((interval between solar cell optimaization is',rest_interval_min,'minutes ...)))')
        
        for t in range (int(round(rest_interval_sec)), 0, -1):
            if (t>10):
                if ( ( t % 10 ) == 0 ):
                    print('time to next measurement =',t,'sec.')
                time.sleep(1)
            else:
                print('time to next measurement =',t,'sec.')
                time.sleep(1)


#==================================================================================================== 
if __name__ == '__main__':     # Program entrance
    print ('PROGRAM IS STARTING ... ')
    try:
        workflow()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        destroy()




