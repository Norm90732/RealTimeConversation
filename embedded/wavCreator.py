import serial
import wave
import numpy as np

def wav():
    intTotal = []
    for element in total:
        intTotal.append((int(element) - 1551) * 16)
    data = np.array(intTotal, dtype =np.int16)
    with wave.open("output.wav", "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(16000)
        wav_file.setnframes(32000)
        wav_file.writeframes(data.tobytes())



# Open serial connection (adjust COM port and baud rate as needed)
ser = serial.Serial('COM4', baudrate=1000000)

print("Connected to", ser.port)
a = 0
counter = 0
current = []
buf = []
total = []
try:
    while True:
        if ser.in_waiting:  # Check if data is available
           counter = 0
           while counter < 16000:
               line = ser.read(4).decode("ASCII")
               buf.append(line)
               counter += 1
           print(buf[0:10])
           a += 1
           a = a%2
           total += buf
           if a == 0:
               wav()
               total = []
           buf = []

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    print("Serial port closed.")



