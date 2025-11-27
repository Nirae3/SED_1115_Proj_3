ADS_MIN_RAW = 9
#converting raw ADS1015 reading to 16-bit value
SCALING_FACTOR = 41.5 # SCALING_FACTOR = 65535 / (  (ADS_MAX)/32  -  (ADS_MIN)/32   ) <- calculated as per

while True:
    try:
        
        desired_pot_value = adc_pot.read_u16() #reads from potentiometer, DUTY CYCLE
        pwm_singal.duty_u16(desired_pot_value) #generate PWM signal
        print(f"desired {desired_pot_value}")

        #uart.write((str(desired_pot_value) + "\n").encode()) #sending the PWM value via UART
        
        measured_signal_value_raw = external_adc.read(0, ADS1015_PWM) #receiving and storing the measured analog value in the external_adc variable. ADS1015 reads analot volatge on AINO0 pin
        #print(f"measured ADC val: {measured_signal_value_raw}")

        time.sleep(0.1)
        uart.write((str(measured_signal_value_raw) + "\n").encode()) #sending the PWM value via UART
        

        adjusted_raw = max(0, measured_signal_value_raw - ADS_MIN_RAW) #setting the maximum signal that can be sent through at a time
        measured_signal_value = int(adjusted_raw * SCALING_FACTOR) #turning th analog values to an integer
        measured_signal_value = min(measured_signal_value, 65535) ##setting the minimum signal that can be sent through at a time
        
        measured_uart_value = read_uart_line(uart) 
        
        #reading the value gotten through uart
#        print(f"desired raw pwm signal: {desired_pot_value: <30} | value i got back from partenr: {uart.read_uart_line(str(measured_signal_value_raw).encode())}")
        print(f"Desired raw PWM: {desired_pot_value :<10} | Scaled version Of what cae back: {measured_uart_value * 40}" ) 

