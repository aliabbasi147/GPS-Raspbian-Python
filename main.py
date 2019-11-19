#!/usr/bin/python
import RPi.GPIO as GPIO
import serial
import pynmea2

from math import radians, cos, sin, asin, sqrt

def parseGPS(str):
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        msg.lat,msg.lat_dir,msg.lon,msg.lon_dir
        homeLat = 33.806016
        homeLong = 72.904142
        latDec = calcLat(msg.lat,msg.lat_dir)
        longDec = calcLong(msg.lon,msg.lon_dir)
        
        print ("Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (msg.timestamp,msg.lat,msg.lat_dir,msg.lon,msg.lon_dir,msg.altitude,msg.altitude_units))
        distance = haversine(homeLat,homeLong,latDec,longDec)
        print ("Calculated Value of LATITUDE: %s --- Longitude: %s --- Distance: %s" % (latDec,longDec,distance))
        return distance
    
def haversine(lat1, lon1, lat2, lon2):

    R = 6372.8 # this is in kilometers.  For Earth radius in miles use 3959.87433 km

    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c * 1000
    
def calcLat(lat, latDirection):
    try:
        decimalDeg = int(float(lat)/100)
        minutesDeg = float(lat)-decimalDeg*100    
        latDec = decimalDeg + minutesDeg/60
        if latDirection == 'S':
            return -1*latDec
        elif latDirection == 'N':
            return latDec
    except ValueError,e:
        print e
    
def calcLong(long, longDirection):
    try:
        decimalDeg = int(float(long)/100)
        minutesDeg = float(long)-decimalDeg*100    
        longDec = decimalDeg + minutesDeg/60
        if longDirection == 'W':
            return -1*longDec
        elif longDirection == 'E':
            return longDec
    except ValueError,e:
        print e
    
def initializeGPIO():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(4, GPIO.OUT)
    GPIO.output(4, GPIO.HIGH)
    
gpioInit = False

serialPort = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.5)
while True:
    str = serialPort.readline()
    calcDistance = parseGPS(str)
    if calcDistance >= 30 and gpioInit == False:
        initializeGPIO()
        gpioInit = True
    elif calcDistance < 30 and gpioInit == True:
        GPIO.cleanup()
        gpioInit = False
